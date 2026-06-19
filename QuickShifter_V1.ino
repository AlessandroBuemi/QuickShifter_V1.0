#include <Arduino.h>
#include <Wire.h>
#include "FastIMU.h"        
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <ArduinoJson.h>
#include <Preferences.h> // Libreria per salvare lo stato sulla Flash dell'ESP32

// --- CONFIGURAZIONE HARDWARE & I2C ---
#define PIN_SENSORE_CAMBIO 14        // Pulsante sul PIN 14 (tocca GND per attivare)
#define PIN_RELE_SHIFTER 12          // Comando Transistor sul PIN 12 (HIGH = Stacca, LOW = Riposo)
#define PIN_BUZZER 13                
#define PIN_LED_INDICATORE 2         // LED di stato integrato (o esterno) Active-HIGH

const int SDA_PIN = 33;              
const int SCL_PIN = 32;
const uint8_t BMI160_ADDR = 0x69;    

// --- OGGETTI FASTIMU ---
BMI160 IMU;
calData calibrazione; 
AccelData accelData;
GyroData gyroData;
bool imuPresente = false;

// --- UUID BLE ---
#define CENTRALINA_SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHAR_RX_UUID            "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHAR_TX_UUID            "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

BLEServer* pServer = nullptr;
BLECharacteristic* pTxCharacteristic = nullptr;
bool dispositivoConnesso = false;

// Timing Shifter 
unsigned long lastDebounceTime = 0;
unsigned long debounceDelay = 180;    
int cutOffTimeCorrente = 75;        
int contatoreCambiate = 0;
bool mappaDinamicaAttiva = false;

// --- IMPOSTAZIONI POP & BANG ---
int modalitaPopBang = 0;              // 0 = Spento, 1 = Soft, 2 = Hard
int popBangCutoff1 = 40;              // Ricevuto dallo Slider 1
int popBangCutoff2 = 40;              // Ricevuto dallo Slider 2
const int PAUSA_INTERMEDIA_BANG = 45; // Pausa intermedia per il doppio lampo del LED

bool sensoreGiaPremuto = false;

// --- GESTIONE STATI SALVATI ---
Preferences prefs;
bool centralinaBloccataLock = false;
bool antiWheelieAttivo = true;        // Abilitazione Anti-Wheelie
bool antifurtoMovimentoAttivo = true; // Sirena Allarme
bool audioGenerale = true;            // Muto generale (Feedback Bip)

// --- LOGICA ALLARME 5 MINUTI ---
bool allarmeInCorso = false;
unsigned long tempoInizioAllarme = 0;
const unsigned long DURATA_ALLARME = 5 * 60 * 1000; // 5 minuti in millisecondi
unsigned long ultimoCambioBuzzerAllarme = 0;
bool statoBuzzerAllarme = false;

// Filtro Software Anti-Wheelie
unsigned long tempoInizioInclinazione = 0;
const unsigned long d_ritardoConfermaImpennata = 150;

// Soglia di movimento Antifurto 
const float SOGLIA_SHAKE_ALLARME = 1.45;

// Telemetria Inerziale
float pitchAngolo = 0.0;
float rollAngolo = 0.0;
int antiWheelieLimitGradi = 30;    

// Picchi massimi
float maxSpeedSessione = 0.0;
float maxRollSessione = 0.0;
float maxPitchSessione = 0.0;
int maxRpmSessione = 0;
float maxGAccSessione = 0.0;
float maxGFrenSessione = 0.0;
float maxGLatSessione = 0.0;

// Odometria
int rpmIstantanei = 0;
int velocitaIstantanea = 0;
float kmTotali = 0.0; float kmTrip = 0.0; float oreTotali = 0.0; float oreTrip = 0.0;

// Funzione per salvare le impostazioni correnti nella memoria Flash dell'ESP32
void salvaStatoSuFlash() {
    prefs.begin("shifter_ecu", false);
    prefs.putBool("lockState", centralinaBloccataLock);
    prefs.putBool("alarmState", antifurtoMovimentoAttivo);
    prefs.putBool("audioGen", audioGenerale);
    prefs.putBool("wheelieState", antiWheelieAttivo); 
    prefs.end();
}

void eseguiBipFeedback(int durata = 80) {
    if (!audioGenerale) return; 
    digitalWrite(PIN_BUZZER, HIGH);
    delay(durata);
    digitalWrite(PIN_BUZZER, LOW);
}

// --- LOGICA MODIFICATA PER TRANSISTOR (Active-HIGH) ---
void impostaStatoTransistor(bool attivaTaglio) {
    if (attivaTaglio) {
        digitalWrite(PIN_RELE_SHIFTER, HIGH);   // <<-- MODIFICATO: HIGH attiva il transistor e stacca la corrente
        digitalWrite(PIN_LED_INDICATORE, HIGH); // Accende il LED di stato
    } else {
        digitalWrite(PIN_RELE_SHIFTER, LOW);    // <<-- MODIFICATO: LOW spegne il transistor (stato di riposo della moto)
        digitalWrite(PIN_LED_INDICATORE, LOW);  // Spegne il LED di stato
    }
}

class CentralinaServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) { dispositivoConnesso = true; };
    void onDisconnect(BLEServer* pServer) {
        dispositivoConnesso = false;
        pServer->getAdvertising()->start(); 
    }
};

class CentralinaRxCallbacks: public BLECharacteristicCallbacks {
    void onWrite(BLECharacteristic *pCharacteristic) {
        String rxValue = String(pCharacteristic->getValue().c_str());
        rxValue.trim(); 

        if (rxValue.length() > 0) {
            int inizioJson = rxValue.indexOf('{');
            int fineJson = rxValue.lastIndexOf('}');
            if (inizioJson == -1 || fineJson == -1 || fineJson < inizioJson) return;
            String jsonPulito = rxValue.substring(inizioJson, fineJson + 1);

            JsonDocument doc;
            DeserializationError error = deserializeJson(doc, jsonPulito);
            
            if (!error) {
                bool parametroCambiato = true;

                if (doc.containsKey("type")) {
                    String type = doc["type"].as<String>();
                    
                    if (type == "slider" && doc.containsKey("value")) {
                        cutOffTimeCorrente = doc["value"].as<int>();
                    }
                    else if (type == "scoppi" && doc.containsKey("value")) {
                        modalitaPopBang = doc["value"].as<int>(); 
                    }
                    else if (type == "set_pop_delay" && doc.containsKey("value")) {
                        popBangCutoff1 = doc["value"].as<int>();
                    }
                    else if (type == "set_pop_intensity" && doc.containsKey("value")) {
                        popBangCutoff2 = doc["value"].as<int>();
                    }
                    else if (type == "ecu_state" && doc.containsKey("status")) {
                        String status = doc["status"].as<String>();
                        if (status == "LOCKED") {
                            centralinaBloccataLock = true;
                            impostaStatoTransistor(true); 
                            salvaStatoSuFlash();
                            
                            if (audioGenerale) {
                                digitalWrite(PIN_BUZZER, HIGH); delay(400); digitalWrite(PIN_BUZZER, LOW);
                            }
                            parametroCambiato = false; 
                        } else {
                            centralinaBloccataLock = false;
                            allarmeInCorso = false; 
                            impostaStatoTransistor(false); 
                            digitalWrite(PIN_BUZZER, LOW);
                            salvaStatoSuFlash();
                            
                            if (audioGenerale) {
                                digitalWrite(PIN_BUZZER, HIGH); delay(80); digitalWrite(PIN_BUZZER, LOW); delay(60);
                                digitalWrite(PIN_BUZZER, HIGH); delay(80); digitalWrite(PIN_BUZZER, LOW);
                            }
                            parametroCambiato = false; 
                        }
                    }
                    else if (type == "toggle_dyn_map" && doc.containsKey("value")) {
                        mappaDinamicaAttiva = doc["value"].as<bool>();
                    }
                    else if (type == "set_wheelie_limit" && doc.containsKey("value")) {
                        antiWheelieLimitGradi = doc["value"].as<int>();
                    }
                    else if (type == "toggle_wheelie_system" && doc.containsKey("value")) {
                        antiWheelieAttivo = doc["value"].as<bool>();
                        salvaStatoSuFlash(); 
                    }
                    else if (type == "toggle_alarm_system" && doc.containsKey("value")) {
                        antifurtoMovimentoAttivo = doc["value"].as<bool>();
                        salvaStatoSuFlash();
                        if (!antifurtoMovimentoAttivo) {
                            allarmeInCorso = false;
                            digitalWrite(PIN_BUZZER, LOW);
                        }
                    }
                    else if (type == "toggle_general_audio" && doc.containsKey("value")) {
                        audioGenerale = doc["value"].as<bool>();
                        salvaStatoSuFlash();
                    }
                    else if (type == "set_km_tot" && doc.containsKey("value")) { kmTotali = doc["value"].as<float>(); }
                    else if (type == "set_km_trip" && doc.containsKey("value")) { kmTrip = doc["value"].as<float>(); }
                    else if (type == "set_h_tot" && doc.containsKey("value")) { oreTotali = doc["value"].as<float>(); }
                    else if (type == "set_h_trip" && doc.containsKey("value")) { oreTrip = doc["value"].as<float>(); }
                    else if (type == "reset_axes") { 
                        pitchAngolo = 0.0; rollAngolo = 0.0;
                        maxRollSessione = 0; maxPitchSessione = 0; maxSpeedSessione = 0; maxRpmSessione = 0;
                        maxGAccSessione = 0; maxGFrenSessione = 0; maxGLatSessione = 0;
                    }
                    else if (type == "reset_trip_km") { kmTrip = 0.0; }
                    else if (type == "reset_trip_hours") { oreTrip = 0.0; }
                    else { parametroCambiato = false; }
                }

                if (parametroCambiato) {
                    eseguiBipFeedback(60); 
                }
            }
        }
    }
};

void eseguiTaglioMotoreShifter() {
    if (centralinaBloccataLock) return; 
    contatoreCambiate++;
    
    int tempoTaglioEffettivo = cutOffTimeCorrente;
    if (mappaDinamicaAttiva) {
        if (rpmIstantanei > 8000) tempoTaglioEffettivo = 55;
        else if (rpmIstantanei > 5000) tempoTaglioEffettivo = 65;
        else tempoTaglioEffettivo = 75;
    }

    if (modalitaPopBang == 1 || modalitaPopBang == 2) { 
        impostaStatoTransistor(true); 
        delay(popBangCutoff1);
        
        impostaStatoTransistor(false); 
        delay(PAUSA_INTERMEDIA_BANG); 
        
        impostaStatoTransistor(true); 
        delay(popBangCutoff2);
        
        impostaStatoTransistor(false);
    } else {
        impostaStatoTransistor(true); 
        delay(tempoTaglioEffettivo);
        impostaStatoTransistor(false); 
    }

    if (dispositivoConnesso) {
        JsonDocument txDoc;
        txDoc["contatore"] = contatoreCambiate;
        txDoc["cutTime"] = (modalitaPopBang > 0) ? (popBangCutoff1 + popBangCutoff2) : tempoTaglioEffettivo;
        String output;
        serializeJson(txDoc, output);
        pTxCharacteristic->setValue(output.c_str());
        pTxCharacteristic->notify();
    }
}

void elaboraCanaliInerzialiFastIMU() {
    if (!imuPresente) return; 
    
    IMU.update();
    IMU.getAccel(&accelData);
    IMU.getGyro(&gyroData);

    pitchAngolo = atan2(accelData.accelX, sqrt(accelData.accelY * accelData.accelY + accelData.accelZ * accelData.accelZ)) * 180.0 / PI;
    rollAngolo = atan2(accelData.accelY, accelData.accelZ) * 180.0 / PI;

    pitchAngolo = abs(pitchAngolo);
    rollAngolo = abs(rollAngolo);

    if (rollAngolo > maxRollSessione) maxRollSessione = rollAngolo;
    if (pitchAngolo > maxPitchSessione) maxPitchSessione = pitchAngolo;
    
    if (accelData.accelX > 0 && accelData.accelX > maxGAccSessione) maxGAccSessione = accelData.accelX;
    if (accelData.accelX < 0 && abs(accelData.accelX) > maxGFrenSessione) maxGFrenSessione = abs(accelData.accelX);
    if (abs(accelData.accelY) > maxGLatSessione) maxGLatSessione = abs(accelData.accelY);

    // --- LOGICA ANTIFURTO DI MOVIMENTO ---
    if (centralinaBloccataLock) {
        if (antifurtoMovimentoAttivo) {
            float accTotale = sqrt(accelData.accelX * accelData.accelX + 
                                   accelData.accelY * accelData.accelY + 
                                   accelData.accelZ * accelData.accelZ);
                                   
            if (!allarmeInCorso && (accTotale > SOGLIA_SHAKE_ALLARME || abs(gyroData.gyroX) > 40.0 || abs(gyroData.gyroY) > 40.0)) {
                allarmeInCorso = true;
                tempoInizioAllarme = millis();
                ultimoCambioBuzzerAllarme = millis();
                statoBuzzerAllarme = true;
                digitalWrite(PIN_BUZZER, HIGH); 
            }
        }
    } else {
        if (allarmeInCorso) {
            allarmeInCorso = false;
            digitalWrite(PIN_BUZZER, LOW);
        }
    }

    if (allarmeInCorso) {
        if (millis() - tempoInizioAllarme >= DURATA_ALLARME) {
            allarmeInCorso = false;
            digitalWrite(PIN_BUZZER, LOW);
        } else {
            if (millis() - ultimoCambioBuzzerAllarme >= 150) {
                statoBuzzerAllarme = !statoBuzzerAllarme;
                digitalWrite(PIN_BUZZER, statoBuzzerAllarme ? HIGH : LOW);
                ultimoCambioBuzzerAllarme = millis();
            }
        }
        return; 
    }

    if (!centralinaBloccataLock && antiWheelieAttivo) {
        if ((pitchAngolo > antiWheelieLimitGradi) && (accelData.accelX > 0.25)) {
            if (tempoInizioInclinazione == 0) {
                tempoInizioInclinazione = millis(); 
            }
            
            if (millis() - tempoInizioInclinazione > d_ritardoConfermaImpennata) {
                impostaStatoTransistor(true); 
                delay(50); 
                impostaStatoTransistor(false);
                tempoInizioInclinazione = 0; 
            }
        } else {
            tempoInizioInclinazione = 0; 
        }
    }
}

void inviaPacchettoCompletoTelemetria() {
    if (!dispositivoConnesso) return;
    
    JsonDocument txDoc;
    
    if (!centralinaBloccataLock) {
        rpmIstantanei = random(0, 2) * random(4000, 11500); 
        velocitaIstantanea = rpmIstantanei > 0 ? random(40, 120) : 0;
        if (velocitaIstantanea > maxSpeedSessione) maxSpeedSessione = velocitaIstantanea;
        if (rpmIstantanei > maxRpmSessione) maxRpmSessione = rpmIstantanei;
    } else {
        rpmIstantanei = 0; velocitaIstantanea = 0;
    }

    txDoc["rpm"] = rpmIstantanei;
    txDoc["speed"] = velocitaIstantanea;
    txDoc["pitch"] = (int)pitchAngolo;
    txDoc["roll"] = (int)rollAngolo;
    txDoc["kmTot"] = kmTotali;
    txDoc["kmTrip"] = kmTrip;
    txDoc["hTot"] = oreTotali;
    txDoc["hTrip"] = oreTrip;

    txDoc["maxSpeed"] = (int)maxSpeedSessione;
    txDoc["maxRoll"] = (int)maxRollSessione;
    txDoc["maxPitch"] = (int)maxPitchSessione;
    txDoc["maxRpm"] = maxRpmSessione;
    txDoc["maxGAcc"] = maxGAccSessione;
    txDoc["maxGFren"] = maxGFrenSessione;
    txDoc["maxGLat"] = maxGLatSessione;
    
    txDoc["awStatus"] = antiWheelieAttivo; 
    txDoc["almStatus"] = antifurtoMovimentoAttivo;
    txDoc["audGenStatus"] = audioGenerale; 

    String output;
    serializeJson(txDoc, output);
    pTxCharacteristic->setValue(output.c_str());
    pTxCharacteristic->notify();
}

void setup() {
    Serial.begin(115200);
    
    pinMode(PIN_RELE_SHIFTER, OUTPUT);
    pinMode(PIN_LED_INDICATORE, OUTPUT);
    pinMode(PIN_BUZZER, OUTPUT);          
    
    digitalWrite(PIN_BUZZER, LOW);
    
    pinMode(PIN_SENSORE_CAMBIO, INPUT_PULLUP);

    // --- RIPRISTINO STATO DA FLASH ---
    prefs.begin("shifter_ecu", true);
    centralinaBloccataLock = prefs.getBool("lockState", false);
    antifurtoMovimentoAttivo = prefs.getBool("alarmState", true);
    audioGenerale = prefs.getBool("audioGen", true);
    antiWheelieAttivo = prefs.getBool("wheelieState", true); 
    prefs.end();

    // Applica subito lo stato iniziale corretto per il transistor
    impostaStatoTransistor(centralinaBloccataLock);

    delay(1000);
    Serial.println("Avvio bus I2C (FastIMU)...");
    Wire.begin(SDA_PIN, SCL_PIN);
    Wire.setClock(100000); 
    
    int err = IMU.init(calibrazione, BMI160_ADDR);
    if (err == 0) {
        imuPresente = true;
        Serial.println("BMI160 configurato via FastIMU con successo!");
        eseguiBipFeedback(150); 
    } else {
        imuPresente = false;
    }

    BLEDevice::init("ESP32S3_SHIFTER_ECU");
    pServer = BLEDevice::createServer();
    pServer->setCallbacks(new CentralinaServerCallbacks());
    BLEService *pService = pServer->createService(CENTRALINA_SERVICE_UUID);
    pTxCharacteristic = pService->createCharacteristic(CHAR_TX_UUID, BLECharacteristic::PROPERTY_NOTIFY);
    pTxCharacteristic->addDescriptor(new BLE2902());
    BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(CHAR_RX_UUID, BLECharacteristic::PROPERTY_WRITE);
    pRxCharacteristic->setCallbacks(new CentralinaRxCallbacks());
    pService->start();
    BLEDevice::getAdvertising()->addServiceUUID(CENTRALINA_SERVICE_UUID);
    BLEDevice::startAdvertising();
}

void loop() {
    int statoSensore = digitalRead(PIN_SENSORE_CAMBIO);

    if (statoSensore == LOW) { 
        unsigned long currentTime = millis();
        if (!sensoreGiaPremuto && (currentTime - lastDebounceTime > debounceDelay)) {
            eseguiTaglioMotoreShifter();
            lastDebounceTime = currentTime;
            sensoreGiaPremuto = true; 
        }
    } else { 
        static unsigned long tempoUltimoRilascio = 0;
        if (millis() - tempoUltimoRilascio > 20) {
            sensoreGiaPremuto = false; 
            tempoUltimoRilascio = millis();
        }
    }

    elaboraCanaliInerzialiFastIMU();

    static unsigned long timerInvioTelemetria = 0;
    if (millis() - timerInvioTelemetria > 200) {
        inviaPacchettoCompletoTelemetria();
        timerInvioTelemetria = millis();

        if (!centralinaBloccataLock && dispositivoConnesso && (rpmIstantanei > 0 || velocitaIstantanea > 0)) {
            kmTrip += 0.01; kmTotali += 0.01;
            oreTrip += (0.2 / 3600.0); oreTotali += (0.2 / 3600.0);
        }
    }
    delay(5); 
}