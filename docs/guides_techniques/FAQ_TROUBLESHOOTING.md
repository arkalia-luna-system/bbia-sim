# FAQ Troubleshooting

> Compatibilité Python: 3.11+
>
> Voir aussi: `docs/references/INDEX_THEMATIQUE.md` et `docs/status.md` (État par axe)

## MuJoCo
- Fenêtre ne s’ouvre pas: utiliser `mjpython` (macOS) ou mode headless `MUJOCO_GL=disable`
- EGL headless: vérifier libGL/GLFW (`sudo apt-get install libglfw3-dev libgl1-mesa-dev`)

## PortAudio / Audio
- Erreur device: désactiver en CI `BBIA_DISABLE_AUDIO=1`
- Sample rate: viser 16kHz; ajuster drivers si mismatch

## WebSockets
- Déconnexions: vérifier proxy/timeouts; réduire fréquence, logs côté serveur

## CORS
- Développement: autoriser `http://localhost:*`
- Production: whitelist stricte; éviter `*` en credentials

## Références
- État par axe: `docs/status.md` → Docs / Onboarding
