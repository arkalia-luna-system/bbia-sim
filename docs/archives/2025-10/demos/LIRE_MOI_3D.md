# 💬🤖 POUR VOIR BBIA EN 3D

## ⚠️ **IMPORTANT - macOS**

Sur macOS, **`launch_passive` ne fonctionne qu'avec `mjpython`** (limitation MuJoCo).

### **Pour voir le viewer 3D sur macOS :**

```bash
# 1. Installer mjpython si pas déjà fait
# (généralement via pip install mujoco-viewer)

# 2. Lancer avec mjpython
mjpython examples/demo_chat_bbia_3d.py
```

### **Alternative si mjpython non disponible :**

```bash
# Sans viewer 3D mais simulation fonctionne
source venv/bin/activate
python examples/demo_chat_bbia_3d.py
```
**Résultat :** Chat fonctionne dans terminal, mais pas de fenêtre 3D

---

## ✅ **CE QUI FONCTIONNE**

### **1. Chat BBIA** ✅
- Fonctionne parfaitement
- Conversation dans terminal
- Historique sauvegardé

### **2. Simulation MuJoCo** ✅
- Robot simule en arrière-plan
- Mouvements selon chat
- Mais viewer ne s'affiche pas sur macOS avec python

### **3. Solution**
Utiliser `mjpython` pour voir le viewer 3D sur macOS !

---

## 🎯 **COMMENT LANCER**

```bash
# Sur macOS avec viewer 3D
mjpython examples/demo_chat_bbia_3d.py

# Sinon (sans viewer mais chat OK)
source venv/bin/activate && python examples/demo_chat_bbia_3d.py
```

---

**Le chat fonctionne, il faut juste mjpython pour voir le viewer 3D sur macOS !** 🍎

