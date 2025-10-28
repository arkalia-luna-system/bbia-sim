# 🖼️ CONTRÔLER LA TAILLE DU VIEWER MUJOCO

**Problème :** La fenêtre MuJoCo s'ouvre toujours en plein écran  
**Solution :** Redimensionner manuellement UNE FOIS

---

## 🎯 **SOLUTION RAPIDE**

### **1. Lancer la démo**
```bash
mjpython examples/demo_chat_bbia_3d.py
```

### **2. Redimensionner la fenêtre**
1. Appuyez **2x sur le bouton vert (-)** en haut à gauche de la fenêtre
2. La fenêtre se réduit
3. **Glissez** la fenêtre où vous voulez
4. **Redimensionnez** avec les coins/bords
5. **La taille sera mémorisée** pour les prochaines fois ! ✅

---

## 💡 **ALTERNATIVE : MODE HEADLESS** (Pas de fenêtre)

Si vous ne voulez pas de fenêtre du tout :

```bash
# Sans viewer (mode console)
python examples/demo_chat_bbia_3d.py
```

**Ou utilisez les autres démos en headless :**
```bash
python examples/demo_emotion_ok.py --headless
python examples/demo_behavior_ok.py --headless  
python examples/demo_vision_ok.py --headless
python examples/demo_voice_ok.py --headless
```

---

## 🔧 **ASTUCE MACOS**

### **Split View automatique :**
1. Ouvrez la démo dans le Terminal
2. Maintenez **Cmd** et glissez la fenêtre MuJoCo vers le bord droit
3. macOS crée automatiquement un **split view**
4. Vous pouvez voir Terminal + Viewer côte à côte ! ✅

### **Mission Control :**
1. Appuyez sur **F3** ou balayez 3 doigts vers le haut
2. Glissez la fenêtre MuJoCo dans un **nouvel espace**
3. Lancez la démo dans un espace dédié

---

## 📝 **NOTES**

- MuJoCo **ne permet pas** de contrôler la taille via code
- La fenêtre **se souvient** de sa dernière taille
- Le **mode headless** évite complètement le problème
- Redimensionner **UNE FOIS** suffit pour toujours

**Le plus simple :** Redimensionnez manuellement la première fois !

