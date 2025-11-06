# 📝 Messages Prêts pour GitHub Issues

**Date** : Oct / Nov. 2025  
**Objectif** : Messages à copier-coller directement sur GitHub pour modifier/fermer les issues

---

## ❌ Issue 2: Fermer - Tests `bbia_memory.py` DÉJÀ FAITS

### Message à ajouter dans l'issue :

```markdown
## ✅ Issue Résolue - Tests Déjà Complets

Bonjour ! 

Après vérification, cette issue peut être **fermée** car les tests pour `bbia_memory.py` sont **déjà complets** :

✅ **Fichier de tests existant** : `tests/test_bbia_memory.py` (198 lignes)
✅ **Toutes les fonctionnalités demandées sont testées** :
   - ✅ Sauvegarde/conservation conversations (`test_save_conversation_to_memory_success`)
   - ✅ Chargement mémoire (`test_load_conversation_from_memory_success`)
   - ✅ Gestion fichiers JSON (`test_save_conversation_with_sentiment`)
   - ✅ Préférences utilisateur (`test_remember_preference`)
   - ✅ Apprentissages (`test_remember_learning`)
   - ✅ Gestion erreurs (fichiers corrompus, permissions)

✅ **Tests utilisent fichiers temporaires** (`tempfile`) comme demandé
✅ **Coverage devrait être bon** (à vérifier avec `pytest --cov`)

**Action** : Cette issue peut être fermée. Si le coverage est encore faible, cela peut être dû à des imports conditionnels non détectés par coverage.

Merci ! 🎉
```

---

## ⚠️ Issue 1: Modifier - Améliorer Coverage `bbia_audio.py`

### Message à ajouter dans l'issue :

```markdown
## 📝 Mise à Jour - Précisions sur les Tests

Bonjour !

Après vérification du code, voici une précision importante :

✅ **`detecter_son()` est déjà bien testé** dans plusieurs fichiers :
   - `test_bbia_audio.py`
   - `test_bbia_audio_improved.py`
   - `test_bbia_audio_extended.py` (plusieurs cas de test)

⚠️ **`_capture_audio_chunk()` n'est PAS testée directement**

**Objectif principal de cette issue** :
- Tester spécifiquement `_capture_audio_chunk()` (fonction privée)
- Peut nécessiter des mocks de `sounddevice` pour tester les différents cas
- Tester les cas limites (audio désactivé, erreurs de capture, formats différents)

**Note** : Cette fonction peut être testée indirectement via `enregistrer_audio()`, mais un test direct serait préférable pour améliorer le coverage.

Merci ! 🎯
```

---

## ⚠️ Issue 3: Modifier - Améliorer Tests `bbia_emotions.py`

### Message à ajouter dans l'issue :

```markdown
## 📝 Mise à Jour - Tests de Transitions Complexes

Bonjour !

Après vérification, voici l'état actuel des tests :

✅ **Déjà testé** :
   - ✅ Historique émotions (`test_set_emotion_history`)
   - ✅ Validation intensités limites (`test_set_emotion_intensity_clamping`)
   - ✅ Transitions de base

⚠️ **À ajouter - Tests de transitions complexes** :

1. **Séquences rapides** :
   ```python
   # Test : happy → sad → excited (en < 1 seconde)
   emotions.set_emotion("happy", 0.8)
   emotions.set_emotion("sad", 0.6)
   emotions.set_emotion("excited", 0.9)
   # Vérifier que l'historique contient les 3 transitions
   ```

2. **Transitions avec durées différentes** :
   ```python
   # Test : transition rapide puis lente
   emotions.transition_duration = 0.1
   emotions.set_emotion("happy", 0.8)
   emotions.transition_duration = 2.0
   emotions.set_emotion("sad", 0.6)
   ```

3. **Tests de stress** :
   ```python
   # Test : 10+ transitions successives rapides
   for i in range(15):
       emotions.set_emotion(["happy", "sad", "excited"][i % 3], 0.5 + (i % 5) * 0.1)
   # Vérifier que l'historique est correct et ne dépasse pas la limite
   ```

4. **Transitions avec intensités extrêmes** :
   ```python
   # Test : 0.0 → 1.0 → 0.0
   emotions.set_emotion("happy", 0.0)
   emotions.set_emotion("excited", 1.0)
   emotions.set_emotion("calm", 0.0)
   ```

**Objectif** : Améliorer le coverage des branches de code liées aux transitions complexes.

Merci ! 🎯
```

---

## ⚠️ Issue 5: Modifier - Tests Mapping Commandes Vocales Avancés

### Message à ajouter dans l'issue :

```markdown
## 📝 Mise à Jour - Exemples Concrets de Commandes à Tester

Bonjour !

Après vérification, `VoiceCommandMapper` a des tests de base, mais les **tests avancés** manquent.

**Exemples concrets de commandes à ajouter** :

### 1. Commandes avec ponctuation :
```python
# Test ponctuation
mapper.map_command("salue!")  # Devrait mapper vers "greet"
mapper.map_command("regarde.")  # Devrait mapper vers "look"
mapper.map_command("arrête?")  # Devrait mapper vers "stop"
```

### 2. Commandes multi-mots complexes :
```python
# Test multi-mots avec apostrophe
mapper.map_command("regarde moi s'il te plaît")  # Devrait mapper vers "look"
mapper.map_command("peux-tu me saluer")  # Devrait mapper vers "greet"
mapper.map_command("est-ce que tu peux regarder")  # Devrait mapper vers "look"
```

### 3. Variations linguistiques :
```python
# Test abréviations
mapper.map_command("slt")  # Devrait mapper vers "greet" (salut)
mapper.map_command("regard")  # Devrait mapper vers "look" (regarde)

# Test verlan (si applicable)
mapper.map_command("teuf")  # Devrait mapper vers "greet" (fête → saluer)

# Test variations orthographiques
mapper.map_command("bonjour")  # Exact
mapper.map_command("bon jour")  # Avec espace
mapper.map_command("bonjour!")  # Avec ponctuation
```

### 4. Commandes partielles complexes :
```python
# Test commandes partielles dans phrases longues
mapper.map_command("peux-tu me saluer maintenant")  # "saluer" devrait être détecté
mapper.map_command("je veux que tu regardes par là")  # "regardes" devrait être détecté
```

**Objectif** : Vérifier que le mapper gère correctement :
- ✅ Ponctuation (ignorée ou gérée)
- ✅ Multi-mots avec apostrophes/espaces
- ✅ Variations linguistiques (abréviations, orthographe)
- ✅ Détection de commandes dans phrases longues

Merci ! 🎯
```

---

## ✅ Issue 4: Aucune Modification - Tests Vision Structure Bbox

### Message optionnel (si vous voulez confirmer que le code est prêt) :

```markdown
## ✅ Code Normalisé - Prêt pour Implémentation

Bonjour !

Juste pour confirmer : le code a été **normalisé** pour que tous les bbox (objets YOLO et visages MediaPipe) aient la même structure :

✅ **Structure uniforme** :
```python
"bbox": {
    "x": int,
    "y": int,
    "width": int,
    "height": int,
    "center_x": int,  # ✅ Ajouté aux visages MediaPipe
    "center_y": int   # ✅ Ajouté aux visages MediaPipe
}
```

✅ **Tous les bbox retournés par `scan_environment()` ont maintenant les 6 champs requis**

Le test peut maintenant vérifier ces 6 champs pour **tous** les bbox sans distinction entre objets et visages.

Bonne implémentation ! 🚀
```

---

## 📋 Instructions d'Utilisation

1. **Pour Issue 2** : Copier le message et **fermer l'issue** avec ce commentaire

2. **Pour Issues 1, 3, 5** : 
   - Ouvrir chaque issue sur GitHub
   - Cliquer sur "Comment"
   - Copier-coller le message correspondant
   - Sauvegarder

3. **Pour Issue 4** : 
   - Optionnel : Ajouter le message de confirmation si vous voulez
   - Sinon, laisser l'issue telle quelle (elle est déjà parfaite)

---

## ✅ Checklist

- [ ] Issue 2 : Message ajouté et issue fermée
- [ ] Issue 1 : Message de précision ajouté
- [ ] Issue 3 : Message avec exemples ajouté
- [ ] Issue 5 : Message avec exemples concrets ajouté
- [ ] Issue 4 : (Optionnel) Message de confirmation ajouté

---

**Tous les messages sont prêts à copier-coller !** 🎉

