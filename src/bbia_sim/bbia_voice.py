"""
Module bbia_voice.py
Synthèse et reconnaissance vocale pour BBIA.
Compatible macOS, simple, portable, testé.

Voix : sélection automatique de la voix la plus proche de Reachy Mini Wireless (féminine, française si possible).
Aucune voix officielle n’est documentée publiquement : cette configuration vise à s’en rapprocher au maximum.
"""

import logging
import unicodedata

import pyttsx3
import speech_recognition as sr

logging.basicConfig(level=logging.INFO)

# Liste des voix féminines douces/enfantines à privilégier sur macOS (ordre de préférence)
VOIX_FEMMES_MAC = [
    "Amelie",  # enfantine, douce
    "Audrey",  # douce
    "Virginie",  # douce
    "Julie",  # classique
]


def get_bbia_voice(engine):
    """
    Force l’utilisation d’une seule voix féminine douce/enfantine sur macOS.
    Prend la première voix dont le nom contient 'Amelie' (toute variante, accent ou non),
    en priorité France (fr_FR), sinon Canada (fr_CA), sinon toute Amelie.
    Si aucune voix n’est trouvée, lève une erreur explicite avec un message d’aide.
    """
    voices = engine.getProperty("voices")

    def normalize(s):
        return (
            unicodedata.normalize("NFKD", s)
            .encode("ASCII", "ignore")
            .decode("ASCII")
            .lower()
        )

    # 1. Prio France
    for v in voices:
        if "amelie" in normalize(v.name) and ("fr_FR" in v.id or "fr-FR" in v.id):
            return v.id
    # 2. Prio Canada
    for v in voices:
        if "amelie" in normalize(v.name) and ("fr_CA" in v.id or "fr-CA" in v.id):
            return v.id
    # 3. Toute Amelie
    for v in voices:
        if "amelie" in normalize(v.name):
            return v.id
    # 4. Sinon, message d’aide
    raise RuntimeError(
        "Aucune voix 'Amélie' n’est installée sur ce Mac. Va dans Préférences Système > Accessibilité > Parole > Voix du système et installe une voix française féminine (ex: Amélie)."
    )


def dire_texte(texte):
    """
    Lit un texte à voix haute (TTS) avec la voix la plus fidèle à Reachy Mini Wireless.
    """
    try:
        logging.info(f"Synthèse vocale : {texte}")
        engine = pyttsx3.init()
        # Sélection automatique de la voix
        voice_id = get_bbia_voice(engine)
        engine.setProperty("voice", voice_id)
        engine.setProperty("rate", 170)  # Vitesse normale
        engine.setProperty("volume", 1.0)
        # Pitch non supporté nativement par pyttsx3, dépend du moteur
        engine.say(texte)
        engine.runAndWait()
    except Exception as e:
        logging.error(f"Erreur de synthèse vocale : {e}")
        raise


def reconnaitre_parole(duree=3, frequence=16000):
    """
    Reconnaît la parole via le micro (STT, français par défaut).
    Retourne le texte reconnu ou None.
    """
    r = sr.Recognizer()
    with sr.Microphone(sample_rate=frequence) as source:
        try:
            logging.info("Écoute du micro pour la reconnaissance vocale...")
            audio = r.listen(source, phrase_time_limit=duree)
            texte = r.recognize_google(audio, language="fr-FR")
            logging.info(f"Texte reconnu : {texte}")
            return texte
        except sr.UnknownValueError:
            logging.warning("Aucune parole reconnue.")
            return None
        except Exception as e:
            logging.error(f"Erreur de reconnaissance vocale : {e}")
            return None


def lister_voix_disponibles():
    """
    Affiche la liste des voix TTS disponibles avec leurs propriétés.
    """
    engine = pyttsx3.init()
    voices = engine.getProperty("voices")
    print("\n[BBIA] Voix TTS disponibles :")
    for idx, v in enumerate(voices):
        try:
            langue = (
                v.languages[0].decode(errors="ignore")
                if hasattr(v.languages[0], "decode")
                else str(v.languages[0])
            )
        except Exception:
            langue = str(v.languages)
        print(
            f"{idx+1}. Nom : {v.name} | Langue : {langue} | Genre : {getattr(v, 'gender', '?')} | ID : {v.id}"
        )
    print(f"Total : {len(voices)} voix trouvées.")
    return voices


if __name__ == "__main__":
    import sys

    if len(sys.argv) > 1 and sys.argv[1] == "list-voices":
        lister_voix_disponibles()
        # Optionnel : démo d'écoute de chaque voix
        if len(sys.argv) > 2 and sys.argv[2] == "demo":
            engine = pyttsx3.init()
            voices = engine.getProperty("voices")
            for v in voices:
                print(f"\nTest voix : {v.name} ({v.id})")
                engine.setProperty("voice", v.id)
                engine.setProperty("rate", 170)
                engine.setProperty("volume", 1.0)
                engine.say(f"Bonjour, je suis la voix {v.name}.")
                engine.runAndWait()
        sys.exit(0)
    print("\n[BBIA VOICE DEMO]")
    print("1. Synthèse vocale : BBIA va parler avec la voix la plus fidèle...")
    engine = pyttsx3.init()
    voice_id = get_bbia_voice(engine)
    print(f"Voix sélectionnée : {voice_id}")
    engine.setProperty("voice", voice_id)
    engine.setProperty("rate", 170)
    engine.setProperty("volume", 1.0)
    demo_texte = "Bonjour, je suis BBIA. Je fonctionne sur votre Mac. Ceci est la voix la plus proche de Reachy Mini Wireless."
    engine.say(demo_texte)
    engine.runAndWait()
    print("2. Reconnaissance vocale : Parlez après le bip...")
    import sys
    import time

    sys.stdout.flush()
    time.sleep(0.5)
    print("Bip ! (parlez)")
    texte = reconnaitre_parole(duree=3, frequence=16000)
    if texte:
        print(f"Vous avez dit : {texte}")
        dire_texte(f"Vous avez dit : {texte}")
    else:
        print("Aucune parole reconnue.")
        dire_texte("Je n'ai rien compris.")
