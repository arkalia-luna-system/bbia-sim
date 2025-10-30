#!/usr/bin/env python3
"""
Demo voix BBIA (pyttsx3) avec voix macOS forcée (ex: Amélie).
Usage:
  source venv/bin/activate
  python scripts/voice_demo.py --text "Bonjour" --voice-id com.apple.voice.compact.fr-CA.Amelie --rate 160
"""
import argparse

import pyttsx3


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--text", default="Bonjour, je suis BBIA.")
    parser.add_argument(
        "--voice-id",
        default="com.apple.voice.compact.fr-CA.Amelie",
        help="ID voix macOS (voir say -v '?')",
    )
    parser.add_argument("--rate", type=int, default=170)
    parser.add_argument("--volume", type=float, default=1.0)
    args = parser.parse_args()

    e = pyttsx3.init()
    e.setProperty("voice", args.voice_id)
    e.setProperty("rate", args.rate)
    e.setProperty("volume", args.volume)
    e.say(args.text)
    e.runAndWait()
    print("[OK] Voix lue avec:", args.voice_id)


if __name__ == "__main__":
    main()
