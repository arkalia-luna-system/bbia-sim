# 🎤 Voix Personnalisée (Offline) – Pipeline Propre et Sûr

**Dernière mise à jour** : 26 Janvier 2026

## Objectif

- Générer des voix “mignonnes”/enfantines ou “douces” en local (offline), sans casser les venv existants.
- Lire les WAV via BBIA (`robot.media.play_audio`) ou activer un backend dédié.

---

### 1) Environnement dédié

```bash
python -m venv venv-voice
source venv-voice/bin/activate
pip install -r requirements/requirements-voice.txt

```

---

### 2) Enregistrer un échantillon de référence (optionnel)

```bash
source venv-voice/bin/activate
python scripts/voice_clone/enregistrer_sample.py --out assets/voice/ref.wav --dur 30

```

Conseils: parle calmement, quelques phrases variées; micro proche, pièce calme.

---

### 3) Générer des WAV avec intonation

```bash
source venv-voice/bin/activate
# Douce
python scripts/voice_clone/generate_voice.py \
  --text "Bonjour… je suis BBIA. Je parle doucement et avec bienveillance." \
  --mode douce --out assets/voice/bbia_douce.wav

# Enfant
python scripts/voice_clone/generate_voice.py \
  --text "Hi hi ! Coucou ! Je suis BBIA ! Trop contente de te voir !" \
  --mode enfant --out assets/voice/bbia_enfant.wav

# Enthousiaste
python scripts/voice_clone/generate_voice.py \
  --text "Salut ! Content de te retrouver ! On commence ?" \
  --mode enthousiaste --out assets/voice/bbia_enthousiaste.wav

```

---

### 4) Lecture fiable

```bash
source venv/bin/activate
python scripts/voice_clone/play_voice.py --file assets/voice/bbia_enfant.wav
# Lister périphériques et choisir la sortie si besoin
python scripts/voice_clone/play_voice.py --list-devices
python scripts/voice_clone/play_voice.py --file assets/voice/bbia_enfant.wav --output 1

```

---

### 5) Activer l’usage automatique dans BBIA (optionnel)

Tu peux router `dire_texte()` vers la génération locale (offline) via une commande externe sûre:

```bash
export OPENVOICE_CMD="python scripts/voice_clone/generate_voice.py --text '{text}' --mode enfant --out '{out}'"
export BBIA_TTS_BACKEND=openvoice

```

Si la commande échoue, BBIA retombe automatiquement sur `pyttsx3` (voix système).

---

### 6) Bonnes pratiques d’intonation

- Scinder en courtes phrases; ajouter micro‑pauses (…)
- Utiliser des interjections (hi hi ! youpi ! oh !)
- Ajuster vitesse/pitch en post‑traitement si nécessaire (ffmpeg/sox)

---

### 7) Tests/Qualité

Avant de committer, lancer:

```bash
source venv/bin/activate
black .
ruff check --fix .
mypy .
bandit -r src/bbia_sim -ll

```

---

### Remarques

- Le script `generate_voice.py` utilise Coqui TTS par défaut (offline). OpenVoice pourra être branché ultérieurement dans ce même pipeline.
- Ce guide n'altère pas le venv principal; il reste sûr pour la simu/robot.

---

**Dernière mise à jour** : 26 Janvier 2026
