# Mise à jour Reachy Mini - Novembre 2025

**Source** : [pollen-robotics/reachy_mini](https://github.com/pollen-robotics/reachy_mini)  
**Version SDK Officiel** : v1.1.1 (Nov 25, 2025)  
**Version BBIA** : 1.4.0

---

## Version SDK

**Dernière version** : v1.1.1 (Nov 25, 2025)

**Vérification** :
```bash
pip show reachy-mini
pip install --upgrade reachy-mini
```

---

## Conformité SDK

### Dépendances

Les dépendances suivantes sont requises :
```toml
"reachy_mini_motor_controller>=1.0.0"
"eclipse-zenoh>=1.4.0"
"reachy-mini-rust-kinematics>=1.0.1"
"cv2_enumerate_cameras>=1.2.1"
"soundfile>=0.13.1"
"huggingface-hub>=0.34.4"
"log-throttling>=0.0.3"
"scipy>=1.15.3"
"asgiref>=3.7.0"
"aiohttp>=3.9.0"
"psutil>=5.9.0"
"jinja2>=3.1.0"
"pyserial>=3.5"
```

### API Conformité

**Endpoints REST** :
- `/api/state/full` - Implémenté
- `/api/state/position` - Implémenté
- `/api/state/joints` - Implémenté
- `/healthz` - Implémenté

**Méthodes SDK** :
- `ReachyMini()` - Conforme
- `create_head_pose()` - Conforme
- `goto_target()` - Conforme
- `look_at_world()` - Conforme
- `look_at_image()` - Conforme

---

**Dernière mise à jour** : 15 Décembre 2025  
**Voir aussi** : [AUDIT_REACHY_MINI_DECEMBRE_2025.md](AUDIT_REACHY_MINI_DECEMBRE_2025.md)
