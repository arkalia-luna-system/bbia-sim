# 🚀 COMMANDES POUR RELEASE v1.4.0

**Date** : 22 novembre 2025  
**Version** : v1.4.0  
**Statut** : ✅ Prêt après vérification CI

---

## ✅ CE QUI EST DÉJÀ FAIT

- ✅ Version mise à jour dans `pyproject.toml` : `1.4.0`
- ✅ CHANGELOG.md : Section `[1.4.0]` créée
- ✅ Documentation mise à jour
- ✅ Tous les fichiers commités et pushés sur `develop`

---

## 📋 COMMANDES À EXÉCUTER (après CI verte)

### 1. Créer le tag v1.4.0

```bash
cd /Volumes/T7/bbia-reachy-sim
git tag v1.4.0
git push origin v1.4.0
```

### 2. Merger sur main

```bash
git checkout main
git merge develop
git push origin main
```

### 3. Vérifier que tout est OK

```bash
# Vérifier que le tag existe
git tag --list | grep v1.4.0

# Vérifier que main est à jour
git log --oneline main -5
```

---

## 🌐 CRÉER GITHUB RELEASE

1. Aller sur GitHub → Repository → **Releases**
2. Cliquer sur **"Draft a new release"**
3. Remplir :
   - **Tag**: `v1.4.0`
   - **Release title**: `Release v1.4.0 - 100% d'exploitation`
   - **Description**: Copier depuis `CHANGELOG.md` section `[1.4.0]`
4. Cliquer sur **"Publish release"**

---

## ✅ VÉRIFICATIONS FINALES

- [ ] Tag v1.4.0 créé et pushé
- [ ] Branch main à jour avec develop
- [ ] GitHub Release créée
- [ ] Tests passent sur main
- [ ] Documentation à jour

---

**Note** : Tous les fichiers sont déjà préparés. Il ne reste que le tag et le merge après vérification CI.

