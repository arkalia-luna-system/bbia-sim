#!/usr/bin/env python3
"""
Script pour ouvrir automatiquement les mockups de test visuel
et le favicon dans le navigateur
"""
import os
import subprocess
import webbrowser
from pathlib import Path


def open_file(file_path):
    """Ouvre un fichier avec l'application par défaut"""
    file_path = Path(file_path).absolute()
    if not file_path.exists():
        print(f"❌ Fichier non trouvé: {file_path}")
        return False

    try:
        # macOS
        if os.name == "posix":
            subprocess.run(["open", str(file_path)], check=True)
        # Linux
        elif os.name == "posix":
            subprocess.run(["xdg-open", str(file_path)], check=True)
        # Windows
        else:
            os.startfile(str(file_path))
        return True
    except Exception as e:
        print(f"⚠️  Erreur lors de l'ouverture: {e}")
        return False


def create_favicon_test_html():
    """Crée un fichier HTML pour tester le favicon"""
    html_content = """<!DOCTYPE html>
<html lang="fr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Test Favicon BBIA - 32x32px</title>
    <style>
        body {
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            max-width: 1200px;
            margin: 0 auto;
            padding: 40px 20px;
            background: #f5f5f5;
        }
        .container {
            background: white;
            padding: 40px;
            border-radius: 8px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        h1 {
            color: #008181;
            margin-bottom: 30px;
        }
        .test-section {
            margin: 30px 0;
            padding: 20px;
            background: #f9f9f9;
            border-radius: 4px;
        }
        .test-section h2 {
            color: #333;
            margin-top: 0;
        }
        .favicon-display {
            display: flex;
            align-items: center;
            gap: 20px;
            margin: 20px 0;
        }
        .favicon-size {
            width: 32px;
            height: 32px;
            border: 1px solid #ddd;
            display: flex;
            align-items: center;
            justify-content: center;
            background: white;
        }
        .favicon-size img {
            width: 32px;
            height: 32px;
        }
        .info {
            background: #e3f2fd;
            padding: 15px;
            border-radius: 4px;
            margin: 20px 0;
        }
        .checklist {
            list-style: none;
            padding: 0;
        }
        .checklist li {
            padding: 10px;
            margin: 5px 0;
            background: white;
            border-left: 4px solid #008181;
        }
        .checklist li:before {
            content: "☐ ";
            margin-right: 10px;
        }
    </style>
    <link rel="icon" type="image/png" href="bbia_favicon_32x32.png">
</head>
<body>
    <div class="container">
        <h1>🎨 Test Favicon BBIA - 32x32px</h1>
        
        <div class="info">
            <strong>Objectif :</strong> Vérifier que le favicon est lisible et reconnaissable à 32x32px
        </div>
        
        <div class="test-section">
            <h2>1. Favicon dans l'onglet du navigateur</h2>
            <p>Vérifiez l'onglet de cette page : le favicon devrait apparaître à côté du titre.</p>
            <p><strong>✅ Est-ce lisible ?</strong> Le logo est-il reconnaissable à cette taille ?</p>
        </div>
        
        <div class="test-section">
            <h2>2. Favicon à taille réelle (32x32px)</h2>
            <div class="favicon-display">
                <div class="favicon-size">
                    <img src="bbia_favicon_32x32.png" alt="Favicon BBIA 32x32">
                </div>
                <div>
                    <p><strong>Taille :</strong> 32x32px (taille réelle du favicon)</p>
                    <p><strong>✅ Questions :</strong></p>
                    <ul>
                        <li>Le robot est-il reconnaissable ?</li>
                        <li>Les détails (yeux, forme) sont-ils visibles ?</li>
                        <li>Le contraste est-il suffisant ?</li>
                    </ul>
                </div>
            </div>
        </div>
        
        <div class="test-section">
            <h2>3. Favicon sur différents fonds</h2>
            <div style="display: flex; gap: 20px; flex-wrap: wrap; margin: 20px 0;">
                <div style="background: white; padding: 20px; border: 1px solid #ddd; text-align: center;">
                    <div class="favicon-size" style="margin: 0 auto 10px;">
                        <img src="bbia_favicon_32x32.png" alt="Favicon sur fond blanc">
                    </div>
                    <p><strong>Fond blanc</strong></p>
                </div>
                <div style="background: #1A1A1A; padding: 20px; border: 1px solid #ddd; text-align: center;">
                    <div class="favicon-size" style="margin: 0 auto 10px;">
                        <img src="bbia_favicon_32x32.png" alt="Favicon sur fond sombre">
                    </div>
                    <p style="color: white;"><strong>Fond sombre</strong></p>
                </div>
                <div style="background: #008181; padding: 20px; border: 1px solid #ddd; text-align: center;">
                    <div class="favicon-size" style="margin: 0 auto 10px;">
                        <img src="bbia_favicon_32x32.png" alt="Favicon sur fond bleu">
                    </div>
                    <p style="color: white;"><strong>Fond bleu</strong></p>
                </div>
            </div>
        </div>
        
        <div class="test-section">
            <h2>4. Checklist de validation</h2>
            <ul class="checklist">
                <li>Favicon visible dans l'onglet du navigateur</li>
                <li>Logo reconnaissable à 32x32px</li>
                <li>Détails visibles (yeux, forme du robot)</li>
                <li>Contraste suffisant sur fond blanc</li>
                <li>Contraste suffisant sur fond sombre</li>
                <li>Contraste suffisant sur fond bleu</li>
            </ul>
        </div>
        
        <div class="test-section">
            <h2>5. Notes</h2>
            <p>Si le favicon n'est pas assez lisible :</p>
            <ul>
                <li>Simplifier davantage le design</li>
                <li>Augmenter le contraste</li>
                <li>Enlever les détails trop fins</li>
            </ul>
        </div>
    </div>
</body>
</html>
"""

    html_path = Path("test_favicon.html").absolute()
    with open(html_path, "w", encoding="utf-8") as f:
        f.write(html_content)

    return html_path


def main():
    """Fonction principale"""
    print("=" * 70)
    print("🎨 OUVERTURE DES TESTS VISUELS")
    print("=" * 70)

    current_dir = Path(".")
    tests_dir = current_dir / "tests_visuels"

    # Vérifier que les mockups existent
    mockups = [
        "mockup_fond_clair.png",
        "mockup_fond_sombre.png",
        "mockup_fond_bleu.png",
        "mockup_fond_noir.png",
    ]

    print("\n📸 Ouverture des mockups de test...")
    opened_count = 0

    for mockup in mockups:
        mockup_path = tests_dir / mockup
        if mockup_path.exists():
            print(f"   ✅ Ouverture: {mockup}")
            if open_file(mockup_path):
                opened_count += 1
        else:
            print(f"   ❌ Non trouvé: {mockup}")

    # Créer et ouvrir le test HTML du favicon
    print("\n🌐 Création du test HTML pour le favicon...")
    html_path = create_favicon_test_html()
    print(f"   ✅ Fichier créé: {html_path}")

    # Ouvrir le favicon dans le navigateur
    favicon_path = current_dir / "bbia_favicon_32x32.png"
    if favicon_path.exists():
        print("\n🔍 Ouverture du favicon dans le navigateur...")
        print(f"   ✅ Fichier: {favicon_path}")

        # Ouvrir le HTML de test
        try:
            webbrowser.open(f"file://{html_path}")
            print("   ✅ Page de test ouverte dans le navigateur")
        except Exception as e:
            print(f"   ⚠️  Erreur: {e}")
            print(f"   💡 Ouvrir manuellement: {html_path}")
    else:
        print(f"   ❌ Favicon non trouvé: {favicon_path}")

    print("\n✅ Résumé:")
    print(f"   • {opened_count}/{len(mockups)} mockups ouverts")
    print("   • Test HTML favicon créé et ouvert")

    print("\n💡 Instructions:")
    print("   1. Vérifier visuellement chaque mockup ouvert")
    print("   2. Vérifier le favicon dans l'onglet du navigateur")
    print("   3. Remplir la checklist dans la page HTML")
    print("   4. Documenter les résultats dans TESTS_VISUELS_RESULTATS.md")


if __name__ == "__main__":
    main()
