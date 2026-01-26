# 🔄 Reflash Pi Reachy Mini depuis macOS

**Dernière mise à jour** : 26 Janvier 2026  
**Compatibilité** : SDK Reachy Mini v1.2.13+  
**Version BBIA** : 1.4.0

## 🎯 Vue d'ensemble

Ce guide explique comment reflasher la carte SD du Raspberry Pi du Reachy Mini depuis macOS. Cette procédure est nécessaire pour mettre à jour le firmware ou restaurer le système.

## 📋 Prérequis

- **macOS** (testé sur macOS 12+)
- **Carte SD** du Reachy Mini (minimum 16 GB recommandé)
- **Adaptateur carte SD** (si nécessaire)
- **Image système** Reachy Mini (téléchargée depuis [Pollen Robotics](https://github.com/pollen-robotics/reachy_mini))

## 🛠️ Outils nécessaires

### 1. Installer `balenaEtcher` ou `dd`

**Option A : balenaEtcher (recommandé)**

```bash
# Installer via Homebrew
brew install --cask balenaetcher

# Ou télécharger depuis https://www.balena.io/etcher/
```

**Option B : `dd` (ligne de commande)**

`dd` est déjà installé sur macOS.

## 📥 Télécharger l'image système

1. **Aller sur le repo officiel** :
   - https://github.com/pollen-robotics/reachy_mini
   - Vérifier la section "Firmware" ou "Releases"

2. **Télécharger l'image** :
   - Format : `.img` ou `.zip`
   - Si `.zip`, extraire avec `unzip`

## 🔧 Méthode 1 : balenaEtcher (Graphique)

### Étapes

1. **Lancer balenaEtcher**
2. **Sélectionner l'image** :
   - Cliquer sur "Flash from file"
   - Choisir le fichier `.img`
3. **Sélectionner la carte SD** :
   - Insérer la carte SD dans l'adaptateur
   - Cliquer sur "Select target"
   - Choisir la carte SD (⚠️ vérifier le nom pour éviter d'écraser le mauvais disque)
4. **Flasher** :
   - Cliquer sur "Flash!"
   - Attendre la fin (peut prendre 10-30 minutes selon la taille)
5. **Vérification** :
   - balenaEtcher vérifie automatiquement l'image

## 🔧 Méthode 2 : `dd` (Ligne de commande)

### ⚠️ ATTENTION

`dd` peut écraser n'importe quel disque. **Vérifiez deux fois** le nom du disque avant de lancer la commande.

### Étapes

1. **Identifier la carte SD** :

```bash
# Lister les disques
diskutil list

# Exemple de sortie :
# /dev/disk2 (external, physical):
#    #:                       TYPE NAME                    SIZE       IDENTIFIER
#    0:     FDisk_partition_scheme                        *15.9 GB    disk2
#    1:                 DOS_FAT_32 NO NAME                 15.9 GB     disk2s1
```

2. **Démonter la carte SD** :

```bash
# Remplacer disk2 par votre disque
diskutil unmountDisk /dev/disk2
```

3. **Flasher l'image** :

```bash
# Remplacer :
# - /path/to/image.img par le chemin de votre image
# - disk2 par votre disque (sans le 's1')
sudo dd if=/path/to/image.img of=/dev/rdisk2 bs=1m status=progress

# Note : Utiliser /dev/rdisk2 (raw disk) est plus rapide que /dev/disk2
```

4. **Vérifier** :

```bash
# Vérifier que l'écriture est terminée
sync

# Vérifier le disque
diskutil list /dev/disk2
```

5. **Éjecter la carte SD** :

```bash
diskutil eject /dev/disk2
```

## 🔍 Dépannage

### Problème : Carte SD non détectée

```bash
# Vérifier les permissions
diskutil list

# Si nécessaire, réinstaller les drivers USB
```

### Problème : Erreur "Resource busy"

```bash
# Forcer le démontage
sudo diskutil unmountDisk force /dev/disk2
```

### Problème : Écriture lente

- Utiliser `/dev/rdisk2` au lieu de `/dev/disk2` (raw disk)
- Vérifier la vitesse de la carte SD (UHS-I recommandé)

### Problème : Image corrompue

```bash
# Vérifier le checksum de l'image
shasum -a 256 image.img

# Comparer avec le checksum fourni par Pollen Robotics
```

## ✅ Vérification post-flash

1. **Réinsérer la carte SD** dans le Reachy Mini
2. **Allumer le robot**
3. **Vérifier la connexion** :

```bash
# Depuis un autre terminal
ping reachy-mini.local

# Ou via SSH
ssh pi@reachy-mini.local
```

4. **Vérifier la version** :

```bash
# Depuis le Pi
cat /etc/os-release
```

## 📚 Références

- [SDK Reachy Mini v1.2.13](https://github.com/pollen-robotics/reachy_mini)
- [Documentation balenaEtcher](https://www.balena.io/etcher/)
- [Documentation macOS diskutil](https://developer.apple.com/library/archive/documentation/SystemAdministration/Conceptual/ManPages_iPhoneOS/man8/diskutil.8.html)

## 🔄 Mise à jour du firmware

Après le reflash, mettre à jour le firmware si nécessaire :

```bash
# Depuis le Pi
sudo apt update
sudo apt upgrade

# Ou via le SDK
pip install --upgrade reachy-mini
```

---

**Dernière mise à jour** : 26 Janvier 2026  
**Compatibilité** : SDK Reachy Mini v1.2.13+
