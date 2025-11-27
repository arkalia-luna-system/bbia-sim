#!/usr/bin/env python3
"""Lecture fiable d'un WAV via sounddevice (choix périphérique optionnel).
Usage:
  source venv/bin/activate
  python scripts/voice_clone/play_voice.py --file assets/voice/out.wav
  # lister périphériques: --list-devices
  # forcer output index: --output 1
"""

import argparse
import os

import sounddevice as sd
import soundfile as sf


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--file", required=True)
    p.add_argument("--list-devices", action="store_true")
    p.add_argument("--output", type=int, default=None)
    args = p.parse_args()

    if args.list_devices:
        print(sd.query_devices())
        return

    if args.output is not None:
        sd.default.device = (None, args.output)

    if not os.path.exists(args.file):
        print(f"[PLAY][ERROR] Fichier introuvable: {args.file}")
        return

    try:
        data, sr = sf.read(args.file, dtype="float32")
        sd.play(data, sr)
        sd.wait()
        print("[PLAY] Terminé")
    except Exception as e:
        print("[PLAY][WARN] Lecture via sounddevice a échoué:", e)
        # Fallback natif macOS si disponible
        try:
            os.system(f"afplay '{args.file}'")
            print("[PLAY] Terminé (fallback afplay)")
        except Exception as e2:
            print("[PLAY][ERROR] Fallback afplay a échoué:", e2)


if __name__ == "__main__":
    main()
