# 📱 Configuration Google Play Store - Guide Complet

**Date** : 27 Novembre 2025  
**Package** : `com.arkalia.cia`  
**Flutter Version** : 3.35.3

---

## ❌ ERREUR COURANTE

```
Error: Google Play Android Developer API has not been used in project 1062485264410 
before or it is disabled. Enable it by visiting 
https://console.developers.google.com/apis/api/androidpublisher.googleapis.com/overview?project=1062485264410 
then retry.
```

---

## ✅ SOLUTION COMPLÈTE

### Étape 1 : Activer l'API Google Play Android Developer

1. **Accéder à la console Google Cloud** :
   - URL directe : https://console.developers.google.com/apis/api/androidpublisher.googleapis.com/overview?project=1062485264410
   - Ou : https://console.developers.google.com → Sélectionner projet `1062485264410`

2. **Activer l'API** :
   - Cliquer sur **"Activer"** ou **"Enable"**
   - Attendre 2-5 minutes pour la propagation

3. **Vérifier l'activation** :
   - L'API doit apparaître comme **"Activée"** (Enabled)
   - Statut : ✅ **API enabled**

---

### Étape 2 : Créer un Service Account

1. **Accéder à IAM & Admin** :
   - https://console.cloud.google.com/iam-admin/serviceaccounts?project=1062485264410

2. **Créer un Service Account** :
   - Cliquer sur **"Créer un compte de service"** (Create Service Account)
   - Nom : `github-actions-play-store`
   - Description : `Service account pour déploiement automatique Play Store via GitHub Actions`

3. **Attribuer les rôles** :
   - Rôle : **"Service Account User"** (minimum requis)
   - Optionnel : **"Editor"** pour plus de permissions

---

### Étape 3 : Créer une clé JSON

1. **Créer une clé** :
   - Sélectionner le service account créé
   - Onglet **"Keys"** → **"Add Key"** → **"Create new key"**
   - Type : **JSON**
   - Télécharger le fichier JSON

2. **⚠️ IMPORTANT** : Ne jamais commiter ce fichier dans Git !

---

### Étape 4 : Configurer les permissions Play Store

1. **Accéder à Google Play Console** :
   - https://play.google.com/console

2. **Aller dans Settings → API access** :
   - Section **"Service accounts"**
   - Cliquer sur **"Link service account"**
   - Sélectionner le service account créé (`github-actions-play-store@1062485264410.iam.gserviceaccount.com`)

3. **Attribuer les permissions** :
   - ✅ **View app information and download bulk reports**
   - ✅ **Manage production releases**
   - ✅ **Manage testing track releases**
   - ✅ **Manage testing track releases (internal, alpha, beta)**

---

### Étape 5 : Configurer les secrets GitHub

1. **Accéder aux secrets du dépôt** :
   - GitHub → Settings → Secrets and variables → Actions

2. **Ajouter le secret** :
   - Nom : `GOOGLE_PLAY_SERVICE_ACCOUNT_JSON`
   - Valeur : **Contenu complet du fichier JSON téléchargé** (copier-coller tout le JSON)
   - Type : Secret

3. **Vérifier le format** :
   ```json
   {
     "type": "service_account",
     "project_id": "1062485264410",
     "private_key_id": "...",
     "private_key": "-----BEGIN PRIVATE KEY-----\n...\n-----END PRIVATE KEY-----\n",
     "client_email": "github-actions-play-store@1062485264410.iam.gserviceaccount.com",
     "client_id": "...",
     "auth_uri": "https://accounts.google.com/o/oauth2/auth",
     "token_uri": "https://oauth2.googleapis.com/token",
     "auth_provider_x509_cert_url": "https://www.googleapis.com/oauth2/v1/certs",
     "client_x509_cert_url": "..."
   }
   ```

---

### Étape 6 : Configurer la signature Android (si nécessaire)

Si vous utilisez une signature personnalisée, ajoutez ces secrets :

- `ANDROID_KEYSTORE_PASSWORD` : Mot de passe du keystore
- `ANDROID_KEY_ALIAS` : Alias de la clé
- `ANDROID_KEY_PASSWORD` : Mot de passe de la clé

---

## 🔍 VÉRIFICATIONS

### Checklist avant déploiement

- [ ] API Google Play Android Developer activée
- [ ] Service Account créé avec clé JSON
- [ ] Service Account lié dans Play Console avec permissions
- [ ] Secret `GOOGLE_PLAY_SERVICE_ACCOUNT_JSON` configuré dans GitHub
- [ ] Format JSON valide (pas d'espaces, sauts de ligne corrects)
- [ ] Attente de 2-5 minutes après activation de l'API

---

## 🚀 UTILISATION

### Déploiement automatique

Le workflow se déclenche automatiquement sur :
- Push sur `main`, `develop`, ou `future`
- Tags `android-v*` (ex: `android-v1.0.0`)

### Déploiement manuel

1. Aller dans **Actions** → **Deploy Android to Google Play**
2. Cliquer sur **"Run workflow"**
3. Sélectionner la branche et la piste (internal, alpha, beta, production)
4. Cliquer sur **"Run workflow"**

---

## ⚠️ DÉPANNAGE

### Erreur : "API not enabled"

**Solution** :
1. Vérifier que l'API est activée : https://console.developers.google.com/apis/api/androidpublisher.googleapis.com/overview?project=1062485264410
2. Attendre 2-5 minutes après activation
3. Relancer le workflow

---

### Erreur : "Service account not linked"

**Solution** :
1. Vérifier que le service account est lié dans Play Console
2. Vérifier les permissions du service account
3. Vérifier que le `client_email` dans le JSON correspond au service account lié

---

### Erreur : "Invalid JSON"

**Solution** :
1. Vérifier que le secret contient le JSON complet (pas juste une partie)
2. Vérifier qu'il n'y a pas d'espaces en début/fin
3. Vérifier que les sauts de ligne dans `private_key` sont corrects (`\n`)

---

### Erreur : "Package name mismatch"

**Solution** :
1. Vérifier que le `packageName` dans le workflow correspond à `com.arkalia.cia`
2. Vérifier dans `android/app/build.gradle` que `applicationId` est correct

---

## 📊 RÉSUMÉ

### Ce qui manquait dans votre configuration

1. ❌ **API non activée** : L'API Google Play Android Developer n'était pas activée
2. ⚠️ **Service Account non configuré** : Pas de service account avec permissions
3. ⚠️ **Secret GitHub manquant** : Le secret `GOOGLE_PLAY_SERVICE_ACCOUNT_JSON` n'était pas configuré
4. ⚠️ **Gestion d'erreur absente** : Pas de fallback si l'API n'est pas activée

### Ce qui est mieux maintenant

1. ✅ **Workflow complet** : Workflow avec toutes les étapes nécessaires
2. ✅ **Gestion d'erreur** : Upload AAB comme artifact en cas d'échec
3. ✅ **Vérifications** : Vérification que le AAB existe avant upload
4. ✅ **Documentation** : Guide complet pour résoudre les problèmes
5. ✅ **Summary GitHub** : Résumé automatique dans GitHub Actions
6. ✅ **Déploiement manuel** : Possibilité de déployer manuellement avec choix de piste

---

## 🔗 LIENS UTILES

- **Console Google Cloud** : https://console.cloud.google.com
- **Play Console** : https://play.google.com/console
- **API Android Publisher** : https://console.developers.google.com/apis/api/androidpublisher.googleapis.com
- **Documentation action** : https://github.com/r0adkll/upload-google-play

---

**Dernière mise à jour** : 27 Novembre 2025

