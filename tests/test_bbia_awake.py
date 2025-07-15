import subprocess
import sys

STEPS = [
    "Lumière blanche faible",
    "Lumière qui s'intensifie doucement",
    "Halo bleu",
    "Respiration simulée : inspiration",
    "Respiration simulée : expiration",
    "Léger son de démarrage",
    "Mouvements de tête lents",
    "Mouvements de bras légers",
    "Expression : sourire doux",
    "Première pensée : 'Je suis là, Athalia.'",
    "complètement réveillé et prêt"
]

def test_bbia_awake_sequence():
    print("\n🧪 Test automatisé : Séquence de réveil BBIA (Python)")
    result = subprocess.run([sys.executable, "src/bbia_sim/bbia_awake.py"], capture_output=True, text=True)
    output = result.stdout
    success = True
    print("\n--- Résultat de la séquence ---\n")
    for step in STEPS:
        if step in output:
            print(f"✅ {step}")
        else:
            print(f"❌ {step} ABSENT !")
            success = False
    print("\n--- Résumé ---")
    if success:
        print("🎉 Toutes les étapes de la séquence de réveil sont présentes !")
    else:
        print("⚠️ Certaines étapes sont manquantes ou incorrectes.")
    assert success, "La séquence de réveil BBIA n'est pas complète ou fidèle."

if __name__ == "__main__":
    test_bbia_awake_sequence() 