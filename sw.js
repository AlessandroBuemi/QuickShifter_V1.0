// Service worker minimo: la sua sola presenza, con un gestore "fetch",
// e' quello che serve a Chrome per offrire la vera installazione (schermo
// intero, senza barra indirizzi) invece della semplice scorciatoia.
// Non mette in cache nulla: l'app resta sempre aggiornata all'apertura.

self.addEventListener('install', (event) => {
  self.skipWaiting();
});

self.addEventListener('activate', (event) => {
  self.clients.claim();
});

self.addEventListener('fetch', (event) => {
  // Lascia passare tutte le richieste direttamente alla rete.
  event.respondWith(fetch(event.request));
});
