#!/usr/bin/env python3
"""
Enregistre un échantillon de référence (mono 16 kHz) pour la voix personnalisée.
Usage:
  source venv-voice-clone/bin/activate  # venv dédié si besoin
  python scripts/voice_clone/enregistrer_sample.py --out assets/voice/ref.wav --dur 30
"""
import argparse

import sounddevice as sd
import soundfile as sf


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--out", default="assets/voice/ref.wav")
    p.add_argument("--dur", type=int, default=30)
    p.add_argument("--sr", type=int, default=16000)
    args = p.parse_args()

    print(f"[REC] Enregistrement {args.dur}s → {args.out} (mono {args.sr}Hz)")
    audio = sd.rec(
        int(args.dur * args.sr), samplerate=args.sr, channels=1, dtype="float32"
    )
    sd.wait()
    sf.write(args.out, audio, args.sr)
    print("[REC] Terminé")


if __name__ == "__main__":
    main()
