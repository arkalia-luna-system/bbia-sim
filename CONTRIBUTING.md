# Guide de contribution

Merci de vouloir contribuer à **BBIA-SIM** (simulation / moteur cognitif Reachy Mini).

Le projet est **sim-first** : sans `BBIA_ALLOW_HARDWARE=1`, pas de mouvement matériel. Ne pas documenter le robot réel comme « 100 % validé » ou le firmware Pollen comme « parfait » sans preuve.

## Comment contribuer

1. **Forkez** le dépôt et clonez votre fork.
2. **Créez une branche** pour chaque fonctionnalité ou correction :

   ```bash
   git checkout -b ma-fonctionnalite
   ```

3. **Décrivez clairement** votre modification dans la Pull Request (PR).
4. **Référencez** les issues concernées si besoin.
5. **Respectez le style de code** (`black`, `ruff` ; `isort` si utilisé localement).
6. **Ajoutez des tests** si possible.
7. **Vérifiez** avant la PR (sous-ensemble pertinent suffit souvent) :

   ```bash
   ruff check src/bbia_sim
   pytest tests/ -q --tb=line
   ```

Python : **3.10+** (extra SDK Reachy **3.11+**).

## Bonnes pratiques

- Code lisible, commenté si nécessaire
- Commits clairs et concis
- Respect de la licence MIT
- Impact robot : cocher la section dédiée du template PR

## Processus de revue

- Toute PR sera relue par un mainteneur
- Des suggestions ou demandes de modification peuvent être faites
- Merci de rester courtois et constructif

---

## 📚 Guides Complémentaires

Pour plus de détails, consultez :

- **[Guide Contributeurs](docs/community/CONTRIBUTION_GUIDE.md)** : Guide détaillé complet
- **[Guide Contribution](docs/community/CONTRIBUTION_GUIDE.md)** : Processus de contribution détaillé
- **[Guide Hugging Face Spaces](docs/community/GUIDE_HUGGINGFACE_SPACES.md)** : Créer des applications publiques
- **[Innovations BBIA](docs/community/INNOVATIONS_BBIA.md)** : Documentation des innovations majeures

---
Contact : arkalia.luna.system@gmail.com
