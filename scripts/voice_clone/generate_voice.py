#!/usr/bin/env python3
"""Génère un WAV avec intonation (douce/enfant/enthousiaste) et support d'une voix
de référence (--ref) pour un rendu personnel via modèle your_tts (Coqui).

Usage:
  source venv-voice/bin/activate
  python scripts/voice_clone/generate_voice.py --text "Coucou !" --mode enfant --out assets/voice/out.wav
  # avec référence (ta voix):
  python scripts/voice_clone/generate_voice.py --ref assets/voice/ref.wav --text "Bonjour" --mode douce --out assets/voice/ta_voix.wav
"""

import argparse
import datetime
import os
import re
import shutil
import subprocess
import tempfile


def _normalize_ascii(text: str) -> str:
    return (
        text.replace("’", "'")
        .replace("“", '"')
        .replace("”", '"')
        .replace("«", '"')
        .replace("»", '"')
    )


def _sanitize_user_text(text: str) -> str:
    """Nettoie le texte utilisateur pour éviter des phrases vides (ex: '!').

    - Supprime espaces superflus
    - Réduit la ponctuation répétée (!!! -> !)
    - Retire ponctuation orpheline en fin/début de texte
    """
    t = text.strip()
    t = re.sub(r"\s+", " ", t)
    t = re.sub(r"([!?\.])\1+", r"\1", t)
    # Enlever ponctuation seule aux extrémités
    t = re.sub(r"^[!?\.]+\s*", "", t)
    t = re.sub(r"\s*[!?\.]+$", "", t)
    return t.strip()


def prosodize(text: str, mode: str) -> str:
    base = _sanitize_user_text(text)
    if mode == "douce":
        return f"{base}. Je parle doucement, avec calme et bienveillance."
    if mode == "enthousiaste":
        return f"{base}! Youpi! C'est génial! Je suis super contente!"
    if mode == "enfant":
        return f"Hi hi! {base}. Je suis BBIA! Trop contente de te voir!"
    return base


def _preprocess_ref_wav(ref_wav: str) -> str | None:
    """Charge et nettoie la référence pour éviter les erreurs d'énergie nulle.

    - Vérifie que le fichier existe et n'est pas silencieux
    - Supprime les silences d'attaque/queue (seuil simple)
    - Ajoute un epsilon si nécessaire pour éviter divide-by-zero

    Retourne le chemin d'un WAV temp prétraité, ou None si invalide.
    """
    try:
        import numpy as np  # type: ignore
        import soundfile as sf  # type: ignore

        # essayer resample si dispo
        try:
            import librosa  # type: ignore
        except Exception:
            librosa = None  # type: ignore

        if not os.path.exists(ref_wav):
            return None

        wav, sr = sf.read(ref_wav, dtype="float32")
        if wav.ndim > 1:
            wav = wav.mean(axis=1)

        # Retirer DC
        if wav.size:
            wav = wav - float(np.mean(wav))

        # Énergie
        max_amp = float(np.max(np.abs(wav))) if wav.size else 0.0
        if max_amp < 1e-6:
            return None  # trop silencieux

        # Trim silences simple
        thr = 0.01
        nz = np.where(np.abs(wav) > thr)[0]
        if nz.size:
            start, end = int(nz[0]), int(nz[-1]) + 1
            wav = wav[start:end]

        # Normalisation RMS sûre (évite divide-by-zero)
        rms = float(np.sqrt(np.mean(np.square(wav))) + 1e-8)
        target_rms = 0.1  # -20 dBFS approx
        wav = wav * (target_rms / rms)
        # Clamp pour éviter clips inattendus
        wav = np.clip(wav, -0.99, 0.99)

        # Resample vers 16000 Hz (encodeur de locuteur your_tts)
        if librosa is not None and sr != 16000:
            try:
                wav = librosa.resample(wav, orig_sr=sr, target_sr=16000)
                sr = 16000
            except Exception:
                pass

        tmp = tempfile.NamedTemporaryFile(suffix=".wav", delete=False)
        tmp.close()
        sf.write(tmp.name, wav.astype("float32"), sr)
        return tmp.name
    except Exception:
        return None


def synthesize_with_coqui(
    text: str,
    out_path: str,
    ref_wav: str | None,
    model_override: str | None = None,
    lang_override: str | None = None,
    force_clone: bool = False,
) -> bool:
    try:
        from TTS.api import TTS  # type: ignore

        use_model = model_override or os.environ.get(
            "BBIA_COQUI_MODEL",
            "tts_models/multilingual/multi-dataset/your_tts",
        )
        user_lang = lang_override or os.environ.get("BBIA_COQUI_LANG", "fr-fr")

        def _model_aware_lang(model_name: str, lang_code: str) -> str:
            mn = (model_name or "").lower()
            lc = (lang_code or "").lower()
            # XTTS v2 attend des codes courts (ex: 'fr')
            if "xtts" in mn:
                if lc in ("fr-fr", "fr_fr"):
                    return "fr"
                return lc or "fr"
            # your_tts préfère souvent 'fr-fr'
            if "your_tts" in mn and lc == "fr":
                return "fr-fr"
            return lc or "fr-fr"

        # types: assurer str non-None
        lang = _model_aware_lang(str(use_model), str(user_lang))

        if ref_wav and os.path.exists(ref_wav):
            cleaned = _preprocess_ref_wav(ref_wav)
            try:
                tts = TTS(use_model)
                tts.tts_to_file(
                    text=text,
                    file_path=out_path,
                    speaker_wav=(cleaned or ref_wav),
                    language=lang,
                )
                _log_event(f"Clonage OK avec modèle={use_model} lang={lang}")
                print(f"[INFO] Modèle: {use_model} (clonage), langue: {lang}")
            except Exception:
                if force_clone:
                    _log_event("Clonage demandé en force et échoué : arrêt")
                    raise
                # Fallback macOS say (voix féminine) si disponible
                if _try_macos_say(
                    text,
                    out_path,
                    os.environ.get("BBIA_SAY_VOICE", "Aurelie"),
                ):
                    print("[INFO] Fallback macOS say (voix): Aurelie")
                    _log_event("Fallback macOS say utilisé (Aurelie)")
                    return True
                # Sinon: Fallback modèle TTS générique (sans clonage)
                fallback_model = "tts_models/fr/css10/vits"
                _log_event(
                    f"Clonage échoué, fallback vers {fallback_model} (sans clonage)",
                )
                print(
                    f"[WARN] Clonage indisponible -> fallback {fallback_model} (voix générique)",
                )
                tts = TTS(fallback_model)
                tts.tts_to_file(text=text, file_path=out_path)
        else:
            # Pas de ref: utiliser modèle FR simple par défaut
            use_model = model_override or os.environ.get(
                "BBIA_COQUI_MODEL",
                "tts_models/fr/css10/vits",
            )
            tts = TTS(use_model)
            tts.tts_to_file(text=text, file_path=out_path)
            print(f"[INFO] Modèle: {use_model} (sans clonage)")
        return True
    except Exception as e:
        print("[ERR] Coqui indisponible:", e)
        return False


def _ensure_log_dir() -> str:
    log_dir = os.path.join("log")
    os.makedirs(log_dir, exist_ok=True)
    return log_dir


def _log_event(message: str) -> None:
    try:
        log_dir = _ensure_log_dir()
        ts = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        with open(os.path.join(log_dir, "voice_gen.log"), "a", encoding="utf-8") as f:
            f.write(f"[{ts}] {message}\n")
    except Exception:
        pass


def _try_macos_say(text: str, out_path: str, voice: str = "Aurelie") -> bool:
    """Essaie de synthétiser via macOS 'say' avec une voix féminine FR.
    Retourne True si succès, False sinon.
    """
    try:
        if shutil.which("say") is None:
            return False
        # say génère AIFF; convertir en WAV via afconvert ou ffmpeg
        with tempfile.NamedTemporaryFile(suffix=".aiff", delete=False) as tmp:
            tmp_path = tmp.name
        cmd = ["say", "-v", voice, "-o", tmp_path, text]
        r = subprocess.run(cmd, check=False, capture_output=True)
        if r.returncode != 0:
            return False
        # Conversion
        converted = False
        if shutil.which("afconvert") is not None:
            r2 = subprocess.run(
                [
                    "afconvert",
                    tmp_path,
                    "-f",
                    "WAVE",
                    "-d",
                    "LEI16",
                    out_path,
                ],
                check=False,
                capture_output=True,
            )
            converted = r2.returncode == 0
        if not converted and shutil.which("ffmpeg") is not None:
            r3 = subprocess.run(
                ["ffmpeg", "-y", "-i", tmp_path, out_path],
                check=False,
                capture_output=True,
            )
            converted = r3.returncode == 0
        try:
            os.remove(tmp_path)
        except Exception:
            pass
        return converted and os.path.exists(out_path)
    except Exception:
        return False


def _cosine_similarity(a, b) -> float:
    import numpy as np  # type: ignore

    an = np.linalg.norm(a) + 1e-8
    bn = np.linalg.norm(b) + 1e-8
    return float(np.dot(a, b) / (an * bn))


def check_voice_similarity(ref_wav: str, gen_wav: str) -> float | None:
    """Renvoie une similarité timbrale (MFCC moyen, cosinus) entre ref et généré.
    1.0 = très proche, 0 = orthogonal, <0 = très différent.
    """
    try:
        import librosa  # type: ignore
        import numpy as np  # type: ignore
        import soundfile as sf  # type: ignore

        if not (os.path.exists(ref_wav) and os.path.exists(gen_wav)):
            return None

        def embed(path: str):
            wav, sr = sf.read(path, dtype="float32")
            if wav.ndim > 1:
                wav = wav.mean(axis=1)
            if sr != 22050:
                wav = librosa.resample(wav, orig_sr=sr, target_sr=22050)
                sr = 22050
            # MFCC + delta moyens
            mfcc = librosa.feature.mfcc(y=wav, sr=sr, n_mfcc=20)
            dmfcc = librosa.feature.delta(mfcc)
            v = np.concatenate([mfcc.mean(axis=1), dmfcc.mean(axis=1)], axis=0)
            return v

        v1 = embed(ref_wav)
        v2 = embed(gen_wav)
        sim = _cosine_similarity(v1, v2)
        return sim
    except Exception as e:
        _log_event(f"check_voice_similarity erreur: {e}")
        return None


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("--text", required=True)
    p.add_argument(
        "--mode",
        choices=["douce", "enfant", "enthousiaste"],
        default="douce",
    )
    p.add_argument("--out", default="assets/voice/out.wav")
    p.add_argument("--ref", default=None, help="WAV de référence (voix personnelle)")
    p.add_argument("--model", default=None, help="Nom du modèle Coqui à utiliser")
    p.add_argument("--lang", default=None, help="Langue (ex: fr-fr)")
    p.add_argument(
        "--check-voice",
        action="store_true",
        help="Évalue la similarité de voix entre --ref et le WAV généré",
    )
    p.add_argument(
        "--force-clone",
        action="store_true",
        help="Échoue si le clonage n'est pas possible (pas de fallback)",
    )
    p.add_argument(
        "--say-voice",
        default=None,
        help="Voix macOS say pour fallback (ex: Aurelie, Amelie)",
    )
    args = p.parse_args()

    os.makedirs(os.path.dirname(args.out), exist_ok=True)
    text = prosodize(_normalize_ascii(args.text), args.mode)
    # Dernier filet de sécurité: supprimer toute phrase qui serait uniquement ponctuation
    # (rare après nos nettoyages, mais on évite un '!' isolé qui casse Coqui)
    text = re.sub(r"(^|\s)[!?](?=\s|$)", " ", text)
    text = re.sub(r"\s+", " ", text).strip()

    if args.say_voice:
        os.environ["BBIA_SAY_VOICE"] = args.say_voice
    _log_event(
        f"Synthèse demandée mode={args.mode} out={args.out} model={args.model or os.environ.get('BBIA_COQUI_MODEL')} lang={args.lang or os.environ.get('BBIA_COQUI_LANG', 'fr-fr')} say_voice={os.environ.get('BBIA_SAY_VOICE')}",
    )

    ok = synthesize_with_coqui(
        text,
        args.out,
        args.ref,
        args.model,
        args.lang,
        args.force_clone,
    )
    if ok:
        print("[OK] WAV généré:", args.out)
        _log_event(f"WAV généré: {args.out}")
        if args.check_voice and args.ref:
            sim = check_voice_similarity(args.ref, args.out)
            if sim is not None:
                print(f"[INFO] Similarité voix (0..1): {sim:.3f}")
                _log_event(f"Similarité voix: {sim:.3f}")
            else:
                print("[WARN] Impossible de calculer la similarité voix")
                _log_event("Impossible de calculer la similarité voix")
    else:
        print("[FAIL] Impossible de générer le WAV (Coqui)")
        _log_event("Échec synthèse Coqui")


if __name__ == "__main__":
    main()
