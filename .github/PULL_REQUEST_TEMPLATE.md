# Pull Request - BBIA-SIM

## 📋 Description
<!-- Description claire de ce que fait cette PR -->

## 🔄 Type de Changement
<!-- Cocher le type principal -->
- [ ] 🐛 **Bug fix** - Correction d'un bug
- [ ] ✨ **Nouvelle fonctionnalité** - Ajout de fonctionnalité
- [ ] 💥 **Breaking change** - Changement cassant la compatibilité
- [ ] 📚 **Documentation** - Mise à jour documentation uniquement
- [ ] ♻️ **Refactoring** - Amélioration du code sans changement fonctionnel
- [ ] ⚡ **Performance** - Amélioration de performance
- [ ] 🧪 **Tests** - Ajout ou correction de tests
- [ ] 🔧 **Configuration** - Changements config, CI, tools
- [ ] 🤖 **Robot** - Spécifique au robot Reachy Mini

## 🎯 Module(s) Modifié(s)
<!-- Cocher les modules impactés -->
- [ ] **bbia_voice** - Synthèse et reconnaissance vocale
- [ ] **bbia_audio** - Gestion audio et son  
- [ ] **bbia_emotions** - Système d'émotions
- [ ] **bbia_vision** - Vision et reconnaissance
- [ ] **bbia_behavior** - Comportements et animations
- [ ] **bbia_awake** - Module principal/orchestrateur
- [ ] **Tests** - Infrastructure de test
- [ ] **Documentation** - Guides et docs
- [ ] **CI/CD** - Workflows et automatisation

## 🧪 Tests Effectués
<!-- Cocher les tests réalisés -->
- [ ] `make test` - Tests unitaires passent
- [ ] `make test-cov` - Couverture maintenue/améliorée
- [ ] `make demo` - Démonstration complète fonctionne
- [ ] Tests audio spécifiques (si applicable)
- [ ] Tests vision spécifiques (si applicable)
- [ ] Test sur robot réel (si disponible)
- [ ] Tests de performance (si applicable)

## 📊 Impact Couverture
<!-- Informations sur la couverture de tests -->
- **Couverture avant**: X%
- **Couverture après**: Y%
- **Nouvelles lignes couvertes**: Z lignes

## 🔗 Issue(s) Liée(s)
<!-- Référencer les issues liées -->
Fixes #(issue_number)
Related to #(issue_number)

## 📸 Démonstration
<!-- Captures, logs, ou exemples de la fonctionnalité -->

### Avant
```
# Code/comportement avant la modification
```

### Après  
```
# Code/comportement après la modification
```

## 🚦 Checklist Pré-Merge
<!-- OBLIGATOIRE - vérifier avant merge -->
- [ ] Le code suit les standards du projet (black, ruff)
- [ ] Les tests passent localement (`make test`)
- [ ] La documentation est mise à jour si nécessaire
- [ ] Les nouveaux fichiers ont les en-têtes appropriés
- [ ] Pas de secrets/credentials dans le code
- [ ] Compatible avec Python 3.10+ (extra SDK Reachy : 3.11+)
- [ ] Si impact robot réel : `BBIA_ALLOW_HARDWARE` / sim-first respectés
- [ ] Testé sur macOS (développement principal) si applicable
- [ ] Pas de claim « robot 100 % validé » / hardware Pollen « parfait » sans preuve

## 🎵 Compatibilité Audio/Vision
<!-- Si applicable -->
- [ ] Compatible avec les différents systèmes audio
- [ ] Gestion d'erreur si matériel indisponible
- [ ] Tests avec et sans matériel physique

## 🤖 Impact Robot Physique
<!-- Évaluer l'impact sur Reachy Mini -->
- [ ] **Aucun impact** - Simulation uniquement
- [ ] **Compatible** - Fonctionne en simulation ET robot
- [ ] **Robot requis** - Nécessite robot physique
- [ ] **Attention** - Peut impacter sécurité robot

## 📋 Notes pour Review
<!-- Informations utiles pour le reviewer -->

### Points d'Attention
- 
- 

### Questions/Doutes
- 
- 

## 🚀 Post-Merge
<!-- Actions à faire après merge -->
- [ ] Mettre à jour la documentation utilisateur
- [ ] Créer/mettre à jour les exemples
- [ ] Informer la communauté des changements
- [ ] Tests supplémentaires sur robot réel (si applicable)
