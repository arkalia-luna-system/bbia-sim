# 🖼️ COMMENT CONTRÔLER LA TAILLE DU VIEWER MUJOCO

**Date :** 28 Octobre 2025  
**Problème :** Fenêtre MuJoCo s'ouvre en plein écran

---

## 🎯 **SOLUTION PRATIQUE**

La meilleure façon de contrôler la fenêtre MuJoCo est **de la redimensionner manuellement après l'ouverture**.

### **Contrôles disponibles :**
1. **Redimensionner** : Cliquez et tirez les bords de la fenêtre
2. **Repositionner** : Glissez la fenêtre où vous voulez
3. **Réduire** : Cliquez sur le bouton vert (-) en haut à gauche
4. **Fermer** : Appuyez sur `ESC` ou fermez la fenêtre

---

## 💡 **CONSEIL PRATIQUE**

**Après le lancement :**
1. La fenêtre s'ouvre en plein écran
2. **Appuyez 2x sur le bouton vert** (-) pour réduire
3. **Glissez la fenêtre** où vous voulez
4. **Redimensionnez** à la taille désirée
5. La taille sera **mémorisée pour la prochaine fois** !

---

## 🔧 **OPTIONS AUTOMATIQUES**

### **1. Utiliser le mode headless** (aucune fenêtre)
```bash
python examples/demo_emotion_ok.py --headless
python examples/demo_behavior_ok.py --headless
python examples/demo_vision_ok.py --headless
python examples/demo_voice_ok.py --headless
```

### **2. Utiliser split view sur macOS**
- Ouvrez une seconde fenêtre (Finder, Terminal, etc.)
- Maintenez `Cmd` et faites glisser la fenêtre vers le bord
- macOS crée automatiquement un split view

### **3. Utiliser Mission Control (macOS)**
- Appuyez sur `F3` ou balayez 3 doigts vers le haut
- Créez des espaces de bureaux différents
- Lancez MuJoCo dans un espace séparé

---

## 🚀 **COMMANDES RECOMMANDÉES**

### **Pour tester sans viewer :**
```bash
source venv/bin/activate
python examples/demo_emotion_ok.py --emotion happy --headless
python examples/demo_behavior_ok.py --behavior wake_up --headless
python examples/demo_vision_ok.py --headless
python examples/demo_voice_ok.py --headless
```

### **Pour voir le viewer 3D :**
```bash
mjpython examples/demo_chat_bbia_3d.py
# Puis redimensionner la fenêtre comme vous voulez !
```

---

## 📝 **NOTES**

- MuJoCo n'offre pas de contrôle direct de la taille de fenêtre via API
- La fenêtre se souvient de sa taille dernière
- Le mode headless est plus stable sur macOS
- Les démos fonctionnent parfaitement en headless

**Le plus simple :** Redimensionnez manuellement la fenêtre une fois, elle se souviendra !

