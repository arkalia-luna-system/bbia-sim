#!/usr/bin/env python3
"""
Test DeepFace - Reconnaissance visage personnalisée + émotions

Usage:
    # Dans venv-vision-py310 (où DeepFace est installé)
    source venv-vision-py310/bin/activate

    # 1. Enregistrer une personne
    python scripts/test_deepface.py --register photo_alice.jpg --name Alice

    # 2. Reconnaître une personne
    python scripts/test_deepface.py --recognize frame.jpg

    # 3. Détecter émotion
    python scripts/test_deepface.py --emotion photo.jpg
"""

import argparse
import sys
from pathlib import Path

# Ajouter src au path
sys.path.insert(0, str(Path(__file__).parent.parent / "src"))

try:
    from bbia_sim.face_recognition import create_face_recognition

    DEEPFACE_AVAILABLE = True
except ImportError:
    print("❌ DeepFace non disponible")
    print("💡 Installer avec: pip install deepface")
    print("   (dans venv-vision-py310 recommandé)")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Test DeepFace BBIA")
    parser.add_argument(
        "--register",
        type=str,
        help="Enregistrer une personne (chemin vers photo)",
    )
    parser.add_argument(
        "--name",
        type=str,
        help="Nom de la personne (requis avec --register)",
    )
    parser.add_argument(
        "--recognize",
        type=str,
        help="Reconnaître une personne (chemin vers image)",
    )
    parser.add_argument(
        "--emotion",
        type=str,
        help="Détecter émotion (chemin vers image)",
    )
    parser.add_argument(
        "--db",
        type=str,
        default="faces_db",
        help="Chemin base de données visages (défaut: faces_db)",
    )
    parser.add_argument(
        "--list",
        action="store_true",
        help="Lister les personnes enregistrées",
    )

    args = parser.parse_args()

    # Créer le module
    face_rec = create_face_recognition(db_path=args.db)
    if not face_rec:
        print("❌ Impossible de créer le module DeepFace")
        return 1

    # Lister personnes
    if args.list:
        persons = face_rec.get_registered_persons()
        print(f"👤 Personnes enregistrées ({len(persons)}):")
        for person in persons:
            print(f"   • {person}")
        return 0

    # Enregistrer une personne
    if args.register:
        if not args.name:
            print("❌ --name requis avec --register")
            return 1

        if not Path(args.register).exists():
            print(f"❌ Photo introuvable: {args.register}")
            return 1

        print(f"📸 Enregistrement de '{args.name}'...")
        success = face_rec.register_person(args.register, args.name)
        if success:
            print(f"✅ '{args.name}' enregistré avec succès!")
        else:
            print(f"❌ Erreur enregistrement de '{args.name}'")
            return 1
        return 0

    # Reconnaître une personne
    if args.recognize:
        if not Path(args.recognize).exists():
            print(f"❌ Image introuvable: {args.recognize}")
            return 1

        print(f"🔍 Reconnaissance dans '{args.recognize}'...")
        result = face_rec.recognize_person(args.recognize, enforce_detection=False)

        if result:
            print(f"✅ Personne reconnue:")
            print(f"   • Nom: {result['name']}")
            print(f"   • Confiance: {result['confidence']:.2%}")
            print(f"   • Distance: {result['distance']:.3f}")
            print(f"   • Modèle: {result['model']}")
        else:
            print("❌ Aucune personne reconnue")
            print("💡 Enregistre d'abord une personne avec --register")
        return 0

    # Détecter émotion
    if args.emotion:
        if not Path(args.emotion).exists():
            print(f"❌ Image introuvable: {args.emotion}")
            return 1

        print(f"😊 Détection émotion dans '{args.emotion}'...")
        result = face_rec.detect_emotion(args.emotion, enforce_detection=False)

        if result:
            print(f"✅ Émotion détectée:")
            print(f"   • Émotion dominante: {result['emotion']}")
            print(f"   • Confiance: {result['confidence']:.2%}")
            print(f"\n   Scores détaillés:")
            for emotion, score in sorted(
                result["scores"].items(), key=lambda x: x[1], reverse=True
            ):
                print(f"      • {emotion}: {score:.1f}%")
        else:
            print("❌ Aucune émotion détectée (pas de visage trouvé)")
        return 0

    # Aucune action, afficher aide
    parser.print_help()
    return 1


if __name__ == "__main__":
    sys.exit(main())
