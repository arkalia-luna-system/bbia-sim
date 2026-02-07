# Issues Pollen ouvertes — impact BBIA (essentiel)

**Dernière mise à jour** : 7 Février 2026  
**Repo** : [pollen-robotics/reachy_mini](https://github.com/pollen-robotics/reachy_mini) — 69 issues ouvertes

## Mitigations BBIA (déjà fait)

| Issue | Sujet | Impact BBIA | Mitigation |
|-------|--------|-------------|------------|
| **#709** | WSL2 timeout Zenoh, config host/IP | Connexion robot depuis WSL2 | `REACHY_ZENOH_CONNECT` ou `BBIA_ZENOH_CONNECT` (ex: `tcp://192.168.1.10:7447`) |
| **#677** | Apps ne se connectent pas Zenoh Wireless (7447) | Robot Wireless sur autre IP | Même env : définir l’IP du robot (ex: `tcp://<IP_ROBOT>:7447`) |
| **#771** | Messages d’erreur caméra (dysfonctionnement apps) | Erreurs caméra peu claires | Messages warning explicites dans `bbia_vision` (vérifier câble, daemon) |

## À savoir (pas de correctif BBIA)

- **#759** Connexion bloquée Lite macOS → Voir [REFLASH_PI_MACOS.md](REFLASH_PI_MACOS.md) et issue Pollen.
- **#764** GStreamer macOS → Doc Pollen ; BBIA utilise WebSocket/OpenCV, pas GStreamer pour le flux principal.
- **#694** Conflit Matplotlib segfault → Si crash avec matplotlib, mettre à jour matplotlib / éviter usage concurrent avec backend GUI.
- **#762 #758 #738** Apps / audio → Côté daemon officiel ; BBIA utilise son propre dashboard et API.

## Variables d’environnement utiles

```bash
# Zenoh (robot sur autre machine ou WSL2)
export REACHY_ZENOH_CONNECT="tcp://192.168.1.10:7447"
# ou
export BBIA_ZENOH_CONNECT="tcp://<IP_ROBOT>:7447"
```
