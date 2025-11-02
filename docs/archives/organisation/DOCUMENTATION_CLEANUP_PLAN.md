# 📋 Plan de Nettoyage Documentation - BBIA-SIM

**Date :** Oct / Oct / Nov. 20255
**Objectif :** Harmoniser, dédoublonner et organiser toute la documentation

---

## 🎯 ÉTAT ACTUEL

### Fichiers MD à la racine (à nettoyer)
- ❌ `ETAT_ACTUEL.md` - État temporaire, doit être archivé
- ❌ `TEST_MAINTENANT.md` - Instructions temporaires, doit être archivé
- ❌ `SYNTHESE_FINALE_OCTOBRE_2025.md` - Doit aller dans docs/archives/
- ❌ `TESTS_MANQUANTS_OCTOBRE_2025.md` - Doit aller dans docs/archives/
- ❌ `RESUME_FINAL_COMPLET_OCTOBRE_2025.md` - Doublon, doit être archivé

### Doublons identifiés dans docs/archives/2025-10/
- ❌ `SYNTHESE_COMPLETE_28OCT2025.md` (doublon de SYNTHESE_FINALE_OCTOBRE_2025.md)
- ❌ `RESUME_FINAL_OCTOBRE_2025.md` (doublon de RESUME_FINAL_COMPLET_OCTOBRE_2025.md)
- ❌ `RAPPORT_FINAL_OCTOBRE_2025.md` (contenu déjà dans SYNTHESE_FINALE)

---

## ✅ PLAN D'ACTION

### Phase 1 : Nettoyage Racine (IMMÉDIAT)

**Action :** Déplacer ou archiver tous les fichiers temporaires

```bash
# Fichiers à archiver dans docs/archives/2025-10/
- ETAT_ACTUEL.md → docs/archives/2025-10/
- TEST_MAINTENANT.md → docs/archives/2025-10/
- TESTS_MANQUANTS_OCTOBRE_2025.md → docs/archives/2025-10/
- RESUME_FINAL_COMPLET_OCTOBRE_2025.md → docs/archives/2025-10/
- SYNTHESE_FINALE_OCTOBRE_2025.md → docs/archives/2025-10/
```

### Phase 2 : Consolidation Documentation

**Créer un fichier unique STATUT_PROJET.md**

Contenu à consolider :
- Info de ETAT_ACTUEL.md (Dashboard lancé)
- Info de TEST_MAINTENANT.md (Tests fonctionnels)
- Info de SYNTHESE_FINALE (Résumé 55 tests créés)
- Info de TESTS_MANQUANTS (Tests manquants prioritaires)

### Phase 3 : Suppression Doublons

**Doublons dans docs/archives/2025-10/ :**
- Supprimer doublons identifiés
- Garder uniquement les plus complets

### Phase 4 : Organisation Finale

**Structure docs/ :**

```
docs/
├── README.md                          # Index principal
├── ARCHITECTURE.md                     # Index architecture
├── ARCHITECTURE_OVERVIEW.md           # Vue d'ensemble
├── ARCHITECTURE_DETAILED.md           # Détails techniques
├── GUIDE_DEBUTANT.md                  # Guide débutant (à compléter)
├── GUIDE_AVANCE.md                    # Guide avancé (à compléter)
├── GUIDE_CHAT_BBIA.md                # Guide chat (actuel)
├── TESTING_GUIDE.md                   # Guide tests (actuel)
├── STATUT_PROJET.md                   # NOUVEAU - Status actuel
├── guides/                           # Guides spécifiques
├── installation/                      # Docs installation
├── simulations/                       # Docs simulation
├── unity/                             # Docs Unity
├── audit/                             # Audit docs
└── archives/                          # Archives historiques
    ├── 2025-10/                      # Docs Oct / Oct / Nov. 20255
    └── phases/                       # Docs phases
```

---

## 🚀 PRIORITÉS

1. ✅ **Immediate :** Nettoyer racine (5 fichiers)
2. ✅ **Urgent :** Créer STATUT_PROJET.md consolidé
3. ✅ **Important :** Vérifier et mettre à jour GUIDE_DEBUTANT.md
4. ✅ **Important :** Vérifier et mettre à jour GUIDE_AVANCE.md
5. ⏳ **Normal :** Organiser archives
6. ⏳ **Normal :** Créer index navigation amélioré

---

## 📊 RÉSULTAT ATTENDU

- ✅ Racine propre (seulement README.md, CHANGELOG.md, etc.)
- ✅ Info consolidée dans STATUT_PROJET.md
- ✅ Documentation organisée et accessible
- ✅ Sans doublons
- ✅ Adaptée à tous niveaux (débutant → expert)

