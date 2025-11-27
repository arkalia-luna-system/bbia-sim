# 📱 Guide d'Installation PWA - BBIA Dashboard

**Date** : 24 Novembre 2025  
**Version** : 1.0

---

## 🎯 Qu'est-ce qu'une PWA ?

Une **Progressive Web App (PWA)** est une application web qui peut être installée sur votre appareil (smartphone, tablette, ordinateur) et fonctionne comme une application native, même hors ligne.

### Avantages de la PWA BBIA

- ✅ **Installation rapide** : Pas besoin d'App Store ou Play Store
- ✅ **Fonctionnement hors ligne** : Service Worker cache les ressources
- ✅ **Icône sur l'écran d'accueil** : Accès rapide au dashboard
- ✅ **Expérience native** : Interface plein écran, sans barre d'adresse
- ✅ **Mise à jour automatique** : Service Worker gère les mises à jour

---

## 📱 Installation sur Android

### Méthode 1 : Via Chrome (Recommandé)

1. **Ouvrir le dashboard** dans Chrome Android
   ```
   http://votre-ip:8000
   ```

2. **Menu Chrome** (3 points en haut à droite)
   - Sélectionner **"Ajouter à l'écran d'accueil"** ou **"Installer l'application"**

3. **Confirmer l'installation**
   - Une popup apparaît : **"Ajouter BBIA à l'écran d'accueil ?"**
   - Cliquer sur **"Ajouter"**

4. **✅ Terminé !**
   - L'icône BBIA apparaît sur l'écran d'accueil
   - Lancer l'app comme une application native

### Méthode 2 : Via le bouton d'installation

1. **Ouvrir le dashboard** dans Chrome Android
2. **Chercher le bouton flottant** "Installer l'app" (en bas à droite)
3. **Cliquer sur le bouton**
4. **Confirmer l'installation** dans la popup

---

## 🍎 Installation sur iOS (iPhone/iPad)

### Via Safari (iOS 11.3+)

1. **Ouvrir le dashboard** dans Safari iOS
   ```
   http://votre-ip:8000
   ```

2. **Bouton de partage** (carré avec flèche)
   - En bas de l'écran Safari

3. **Faire défiler** jusqu'à **"Sur l'écran d'accueil"**
   - Ou **"Ajouter à l'écran d'accueil"**

4. **Personnaliser le nom** (optionnel)
   - Par défaut : "BBIA Robot Control"

5. **Cliquer sur "Ajouter"** (en haut à droite)

6. **✅ Terminé !**
   - L'icône BBIA apparaît sur l'écran d'accueil
   - Lancer l'app comme une application native

### Note iOS

- ⚠️ **Service Worker limité** : iOS a des limitations sur le cache offline
- ⚠️ **Pas de popup automatique** : Il faut utiliser le menu Safari
- ✅ **Fonctionne bien** : L'app s'ouvre en plein écran

---

## 💻 Installation sur Desktop (Chrome/Edge)

### Via Chrome/Edge

1. **Ouvrir le dashboard** dans Chrome ou Edge
   ```
   http://localhost:8000
   ```

2. **Icône d'installation** dans la barre d'adresse
   - Chercher l'icône **"Installer"** (carré avec flèche) à droite de l'URL

3. **Cliquer sur "Installer"**
   - Une popup apparaît : **"Installer BBIA Robot Control ?"**

4. **Confirmer l'installation**
   - Cliquer sur **"Installer"**

5. **✅ Terminé !**
   - L'app s'ouvre dans une fenêtre dédiée (sans barre d'adresse)
   - Icône dans le menu Démarrer (Windows) ou Applications (macOS)

### Via le bouton d'installation

1. **Ouvrir le dashboard**
2. **Chercher le bouton flottant** "Installer l'app" (en bas à droite)
3. **Cliquer sur le bouton**
4. **Confirmer l'installation** dans la popup

---

## 🔧 Vérification de l'Installation

### Vérifier que la PWA est installée

1. **Ouvrir l'app** depuis l'écran d'accueil
2. **Vérifier l'interface** :
   - ✅ Pas de barre d'adresse (plein écran)
   - ✅ Icône BBIA dans la barre des tâches
   - ✅ Titre "BBIA Robot Control"

### Vérifier le Service Worker

1. **Ouvrir DevTools** (F12)
2. **Onglet "Application"** (Chrome) ou **"Storage"** (Firefox)
3. **Section "Service Workers"**
   - ✅ Statut : **"activated and is running"**
   - ✅ Source : `sw.js`

### Vérifier le Cache

1. **DevTools > Application > Cache Storage**
2. **Vérifier les caches** :
   - ✅ `bbia-static-v1` : Ressources statiques
   - ✅ `bbia-api-v1` : Réponses API (si disponibles)

---

## 🐛 Dépannage

### L'app ne s'installe pas

**Problème** : Le bouton "Installer" n'apparaît pas

**Solutions** :
1. ✅ Vérifier que vous êtes en **HTTPS** ou **localhost**
2. ✅ Vérifier que le **manifest.json** est accessible
3. ✅ Vérifier que le **Service Worker** est enregistré
4. ✅ Vider le cache du navigateur (Ctrl+Shift+Delete)

### Le Service Worker ne fonctionne pas

**Problème** : L'app ne fonctionne pas hors ligne

**Solutions** :
1. ✅ Vérifier que `sw.js` est accessible
2. ✅ Vérifier la console DevTools pour les erreurs
3. ✅ Désinscrire et réinscrire le Service Worker :
   ```javascript
   // Dans la console DevTools
   navigator.serviceWorker.getRegistrations().then(registrations => {
       registrations.forEach(reg => reg.unregister());
   });
   // Recharger la page
   ```

### L'app ne se met pas à jour

**Problème** : L'app affiche une ancienne version

**Solutions** :
1. ✅ Forcer la mise à jour :
   - Fermer complètement l'app
   - La rouvrir
2. ✅ Vider le cache :
   - DevTools > Application > Clear Storage
   - Cliquer sur "Clear site data"

---

## 📚 Ressources

### Documentation

- **Manifest PWA** : `src/bbia_sim/daemon/app/dashboard/static/manifest.json`
- **Service Worker** : `src/bbia_sim/daemon/app/dashboard/static/sw.js`
- **Script d'installation** : `src/bbia_sim/daemon/app/dashboard/static/js/pwa_install.js`

### Liens Utiles

- [MDN - Progressive Web Apps](https://developer.mozilla.org/en-US/docs/Web/Progressive_web_apps)
- [Web.dev - PWA](https://web.dev/progressive-web-apps/)
- [Can I Use - Service Workers](https://caniuse.com/serviceworkers)

---

## ✅ Checklist d'Installation

### Avant l'installation

- [ ] Dashboard accessible (HTTP/HTTPS)
- [ ] Manifest.json accessible (`/static/manifest.json`)
- [ ] Service Worker accessible (`/static/sw.js`)
- [ ] Icônes PWA présentes (192x192, 512x512)

### Après l'installation

- [ ] App s'ouvre en plein écran
- [ ] Icône sur l'écran d'accueil
- [ ] Service Worker activé
- [ ] Cache fonctionnel (vérifier DevTools)
- [ ] Mode offline fonctionne (déconnecter réseau, recharger)

---

**Dernière mise à jour** : 24 Novembre 2025  
**Version BBIA** : 1.3.2

