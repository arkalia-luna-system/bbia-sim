// Service Worker pour BBIA Dashboard PWA
// Version: 1.0.0

const CACHE_NAME = 'bbia-dashboard-v1';
const STATIC_CACHE = 'bbia-static-v1';
const API_CACHE = 'bbia-api-v1';

// Fichiers à mettre en cache au démarrage
const STATIC_FILES = [
  '/',
  '/static/style.css',
  '/static/js/daemon.js',
  '/static/js/media.js',
  '/static/js/waveform.js',
  '/static/js/robot_3d.js',
  '/static/js/fps_display.js',
  '/static/js/apps.js',
  '/static/js/appstore.js',
  '/static/js/move_player.js',
  '/static/images/background-blurred.svg',
  '/static/manifest.json'
];

// Installation du Service Worker
self.addEventListener('install', (event) => {
  console.log('[SW] Installation du Service Worker');
  
  event.waitUntil(
    caches.open(STATIC_CACHE).then((cache) => {
      console.log('[SW] Mise en cache des fichiers statiques');
      return cache.addAll(STATIC_FILES).catch((err) => {
        console.warn('[SW] Erreur lors de la mise en cache:', err);
        // Continuer même si certains fichiers échouent
        return Promise.resolve();
      });
    })
  );
  
  // Forcer l'activation immédiate
  self.skipWaiting();
});

// Activation du Service Worker
self.addEventListener('activate', (event) => {
  console.log('[SW] Activation du Service Worker');
  
  event.waitUntil(
    caches.keys().then((cacheNames) => {
      return Promise.all(
        cacheNames.map((cacheName) => {
          // Supprimer les anciens caches
          if (cacheName !== STATIC_CACHE && cacheName !== API_CACHE) {
            console.log('[SW] Suppression ancien cache:', cacheName);
            return caches.delete(cacheName);
          }
        })
      );
    })
  );
  
  // Prendre le contrôle immédiatement
  return self.clients.claim();
});

// Stratégie de cache: Cache First pour statiques, Network First pour API
self.addEventListener('fetch', (event) => {
  const { request } = event;
  const url = new URL(request.url);
  
  // Ignorer les requêtes non-GET
  if (request.method !== 'GET') {
    return;
  }
  
  // WebSocket: pas de cache
  if (url.protocol === 'ws:' || url.protocol === 'wss:') {
    return;
  }
  
  // API REST: Network First avec cache fallback
  if (url.pathname.startsWith('/api/')) {
    event.respondWith(
      fetch(request)
        .then((response) => {
          // Mettre en cache les réponses réussies
          if (response.status === 200) {
            const responseClone = response.clone();
            caches.open(API_CACHE).then((cache) => {
              cache.put(request, responseClone);
            });
          }
          return response;
        })
        .catch(() => {
          // Fallback: utiliser le cache si réseau indisponible
          return caches.match(request).then((cachedResponse) => {
            if (cachedResponse) {
              return cachedResponse;
            }
            // Retourner une réponse d'erreur si pas de cache
            return new Response(
              JSON.stringify({ error: 'Offline - No cached data' }),
              {
                status: 503,
                headers: { 'Content-Type': 'application/json' }
              }
            );
          });
        })
    );
    return;
  }
  
  // Fichiers statiques: Cache First
  event.respondWith(
    caches.match(request).then((cachedResponse) => {
      if (cachedResponse) {
        return cachedResponse;
      }
      
      // Si pas en cache, récupérer du réseau
      return fetch(request).then((response) => {
        // Mettre en cache seulement les réponses réussies
        if (response.status === 200) {
          const responseClone = response.clone();
          caches.open(STATIC_CACHE).then((cache) => {
            cache.put(request, responseClone);
          });
        }
        return response;
      });
    })
  );
});

// Gestion des messages depuis le client
self.addEventListener('message', (event) => {
  if (event.data && event.data.type === 'SKIP_WAITING') {
    self.skipWaiting();
  }
  
  if (event.data && event.data.type === 'CLEAR_CACHE') {
    event.waitUntil(
      caches.keys().then((cacheNames) => {
        return Promise.all(
          cacheNames.map((cacheName) => caches.delete(cacheName))
        );
      })
    );
  }
});

