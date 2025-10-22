import time

from reachy import Reachy, parts

try:
    # Connexion à la simulation web
    r = Reachy(
        right_arm=parts.RightArm(io="ws", hand="force_gripper"),
        left_arm=parts.LeftArm(io="ws", hand="force_gripper"),
    )
    print("✨ Connectée à la simulation Web !")
    time.sleep(1)

    # Test du bras droit
    if hasattr(r, "right_arm") and r.right_arm is not None:
        try:
            r.right_arm.elbow_pitch.goal_position = -80
            print("➡️  Mouvement du coude droit envoyé.")
            time.sleep(2)
        except Exception as e:
            print(f"❌ Erreur lors du mouvement du bras droit : {e}")
    else:
        print("⚠️  Le bras droit n'est pas disponible dans cette simulation.")

    # Test de la tête
    if hasattr(r, "head") and r.head is not None:
        try:
            r.head.yaw.goal_position = 30
            print("➡️  Mouvement de la tête envoyé.")
            time.sleep(2)
        except Exception as e:
            print(f"❌ Erreur lors du mouvement de la tête : {e}")
    else:
        print("⚠️  La tête n'est pas disponible dans cette simulation.")

    print("🎉 Test de mouvement terminé.")

except Exception as e:
    print(f"❌ Impossible de se connecter à la simulation Web Reachy : {e}")
    print(
        "Vérifie que le simulateur web est bien lancé et que tu as cliqué sur 'Connect'."
    )
