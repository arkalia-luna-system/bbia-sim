#!/usr/bin/env python3
"""Script automatique pour reflasher les moteurs - Essaie toutes les options."""

import subprocess
import time

print("🔄 REFLASH AUTOMATIQUE - Reachy Mini")
print("=" * 60)
print()

# Attendre un peu pour que le système détecte le port USB
print("⏳ Attente 3 secondes pour détection USB...")
time.sleep(3)

# Essayer de trouver le port
print("🔍 Recherche du port série...")
try:
    from reachy_mini.daemon.utils import find_serial_port

    # Essayer version Lite (USB)
    ports_lite = find_serial_port(wireless_version=False)
    if ports_lite:
        print(f"✅ Port USB trouvé: {ports_lite[0]}")
        port = ports_lite[0]
    else:
        # Essayer version Wireless
        ports_wireless = find_serial_port(wireless_version=True)
        if ports_wireless:
            print(f"✅ Port Wireless trouvé: {ports_wireless[0]}")
            port = ports_wireless[0]
        else:
            print("❌ Aucun port trouvé automatiquement")
            print("💡 Essayons avec les ports communs...")
            # Ports communs à essayer
            common_ports = ["/dev/ttyAMA3", "/dev/ttyUSB0", "/dev/tty.usbserial-*"]
            port = None
            for p in common_ports:
                print(f"   Test: {p}")
except Exception as e:
    print(f"⚠️  Erreur détection: {e}")
    port = None

# Si port trouvé, lancer le reflash
if port:
    print(f"\n🚀 Lancement reflash sur {port}...")
    try:
        result = subprocess.run(
            ["reachy-mini-reflash-motors", "--serialport", port], check=False
        )
        if result.returncode == 0:
            print("\n✅ Reflash réussi!")
        else:
            print(f"\n⚠️  Code retour: {result.returncode}")
    except Exception as e:
        print(f"❌ Erreur: {e}")
else:
    print("\n❌ Impossible de trouver le port automatiquement")
    print("\n💡 Solutions:")
    print("   1. Vérifier que le robot est bien branché en USB")
    print("   2. Attendre quelques secondes et relancer")
    print("   3. Essayer manuellement: reachy-mini-reflash-motors")
    print("   4. Se connecter en SSH au robot et lancer le script là-bas")
