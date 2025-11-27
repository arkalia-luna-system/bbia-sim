# 🎬 Guide Vidéos de Démonstration BBIA-SIM

**Date** : 26 Novembre 2025  
**Objectif** : Créer des vidéos courtes (30-60 secondes) pour démontrer chaque comportement et fonctionnalité

---

## 📋 Structure Recommandée

### Format Standard

- **Durée** : 30-60 secondes maximum
- **Format** : MP4, 1080p minimum
- **Audio** : Voix off ou sous-titres
- **Structure** :
  1. Introduction (5s) : "BBIA-SIM - [Nom Comportement]"
  2. Démonstration (20-45s) : Comportement en action
  3. Conclusion (5s) : Lien GitHub, badge qualité

---

## 🎯 Vidéos par Comportement (21 comportements)

### Comportements de Base (7)

#### 1. Wake Up (Réveil)
- **Durée** : 30s
- **Contenu** :
  - Robot en position "sleep"
  - Commande `wake_up`
  - Transition vers position neutre
  - Animation antennes
- **Script** : "BBIA se réveille et prend une position neutre, prêt à interagir."

#### 2. Greeting (Salutation)
- **Durée** : 35s
- **Contenu** :
  - Utilisateur entre dans le champ de vision
  - Robot détecte présence
  - Mouvement de salutation (antennes + tête)
  - Message vocal "Bonjour !"
- **Script** : "BBIA détecte votre présence et vous salue avec une animation amicale."

#### 3. Emotional Response (Réponse Émotionnelle)
- **Durée** : 40s
- **Contenu** :
  - Utilisateur dit "Tu es génial !"
  - Robot passe à l'émotion "happy"
  - Animation correspondante
  - Réponse vocale positive
- **Script** : "BBIA réagit émotionnellement aux compliments avec 12 émotions disponibles."

#### 4. Vision Tracking (Suivi Visuel)
- **Durée** : 45s
- **Contenu** :
  - Objet en mouvement
  - Robot suit l'objet avec la tête
  - Détection YOLO visible (overlay)
  - Suivi fluide
- **Script** : "BBIA suit les objets en temps réel grâce à YOLOv8n et la cinématique inverse."

#### 5. Conversation (Conversation LLM)
- **Durée** : 50s
- **Contenu** :
  - Utilisateur : "Raconte-moi une blague"
  - Robot génère réponse avec LLM
  - Réponse vocale
  - Animation conversationnelle
- **Script** : "BBIA utilise Phi-2 ou TinyLlama pour des conversations intelligentes et contextuelles."

#### 6. Antenna Animation (Animation Antennes)
- **Durée** : 30s
- **Contenu** :
  - Différentes animations d'antennes
  - Synchronisation avec émotions
  - Transitions fluides
- **Script** : "Les antennes de BBIA s'animent selon l'émotion et le contexte."

#### 7. Hide (Cacher)
- **Durée** : 35s
- **Contenu** :
  - Robot en position normale
  - Commande "hide"
  - Mouvement de protection (antennes baissées, tête baissée)
  - Message "Je me cache"
- **Script** : "BBIA peut se cacher avec une animation protectrice."

---

### Comportements Avancés (14)

#### 8. Follow Face (Suivi Visage)
- **Durée** : 40s
- **Contenu** :
  - Détection visage avec MediaPipe
  - Suivi tête utilisateur
  - Mouvement fluide
- **Script** : "BBIA suit votre visage en temps réel avec MediaPipe."

#### 9. Follow Object (Suivi Objet)
- **Durée** : 45s
- **Contenu** :
  - Objet coloré en mouvement
  - Détection YOLO
  - Suivi précis
- **Script** : "BBIA suit n'importe quel objet détecté par YOLO."

#### 10. Dance (Danse)
- **Durée** : 50s
- **Contenu** :
  - Séquence de mouvements rythmés
  - Animation complète (tête + antennes)
  - Synchronisation musique (optionnel)
- **Script** : "BBIA peut danser avec des séquences de mouvements préprogrammées."

#### 11. Emotion Show (Spectacle d'Émotions)
- **Durée** : 60s
- **Contenu** :
  - Défilé des 12 émotions
  - Transitions fluides
  - Animation pour chaque émotion
- **Script** : "BBIA peut présenter toutes ses 12 émotions en séquence."

#### 12. Photo Booth (Photobooth)
- **Durée** : 45s
- **Contenu** :
  - Détection visage
  - Compte à rebours "3, 2, 1"
  - Capture photo
  - Animation de confirmation
- **Script** : "BBIA peut prendre des photos automatiquement quand il détecte un visage."

#### 13. Storytelling (Conte)
- **Durée** : 55s
- **Contenu** :
  - Utilisateur : "Raconte-moi une histoire"
  - Génération histoire avec LLM
  - Narration vocale
  - Animations synchronisées
- **Script** : "BBIA raconte des histoires générées par IA avec animations."

#### 14. Teaching (Enseignement)
- **Durée** : 50s
- **Contenu** :
  - Utilisateur : "Explique-moi Python"
  - Explication générée par LLM
  - Animations pédagogiques
- **Script** : "BBIA peut enseigner des concepts avec des explications générées par IA."

#### 15. Game (Jeu)
- **Durée** : 60s
- **Contenu** :
  - Jeu interactif (ex: devinette)
  - Réponses utilisateur
  - Feedback robot
  - Score affiché
- **Script** : "BBIA peut jouer à des jeux interactifs avec vous."

#### 16. Meditation (Méditation)
- **Durée** : 50s
- **Contenu** :
  - Position méditative
  - Respiration synchronisée (antennes)
  - Ambiance calme
- **Script** : "BBIA guide des sessions de méditation avec animations apaisantes."

#### 17. Exercise (Exercice)
- **Durée** : 55s
- **Contenu** :
  - Exercices physiques guidés
  - Comptage répétitions
  - Encouragements
- **Script** : "BBIA peut guider des exercices physiques avec animations."

#### 18. Alarm Clock (Réveil)
- **Durée** : 40s
- **Contenu** :
  - Alarme programmée
  - Réveil avec animation
  - Message vocal
- **Script** : "BBIA peut servir de réveil intelligent avec animations."

#### 19. Weather Report (Météo)
- **Durée** : 45s
- **Contenu** :
  - Utilisateur : "Quel temps fait-il ?"
  - Récupération données météo
  - Rapport vocal
  - Animation selon météo
- **Script** : "BBIA peut donner la météo avec animations contextuelles."

#### 20. News Reader (Lecteur de Nouvelles)
- **Durée** : 50s
- **Contenu** :
  - Utilisateur : "Lis-moi les nouvelles"
  - Récupération actualités
  - Lecture vocale
  - Animations informatives
- **Script** : "BBIA peut lire les actualités avec synthèse vocale."

#### 21. Music Reaction (Réaction Musicale)
- **Durée** : 45s
- **Contenu** :
  - Détection musique
  - Analyse rythme
  - Animations synchronisées
- **Script** : "BBIA réagit à la musique avec des animations rythmées."

---

## 🎥 Vidéos Techniques

### Architecture Backend Unifié
- **Durée** : 60s
- **Contenu** :
  - Démonstration simulation MuJoCo
  - Bascule vers robot réel (si disponible)
  - Même code, deux backends
- **Script** : "BBIA-SIM utilise une architecture backend unifié : même code pour simulation et hardware réel."

### Conformité SDK 100%
- **Durée** : 45s
- **Contenu** :
  - Affichage tests conformité (47 tests)
  - Démonstration méthodes SDK
  - Comparaison avec SDK officiel
- **Script** : "BBIA-SIM est 100% conforme au SDK officiel Pollen Robotics avec 47 tests validés."

### Tests et Coverage
- **Durée** : 40s
- **Contenu** :
  - Exécution tests pytest
  - Affichage coverage 68.86%
  - Badge CI/CD vert
- **Script** : "1,743 tests automatisés avec coverage 68.86% et CI/CD complet."

---

## 📝 Checklist Production Vidéo

### Pré-production
- [ ] Script écrit pour chaque vidéo
- [ ] Scénario validé
- [ ] Matériel préparé (robot, environnement)
- [ ] Outils d'enregistrement configurés

### Production
- [ ] Enregistrement vidéo (1080p minimum)
- [ ] Enregistrement audio (voix off ou sous-titres)
- [ ] Captures d'écran code (si nécessaire)
- [ ] Animations fluides

### Post-production
- [ ] Montage (30-60s)
- [ ] Ajout sous-titres (si nécessaire)
- [ ] Ajout logo/branding
- [ ] Compression optimale
- [ ] Thumbnail attractif

### Publication
- [ ] Upload YouTube
- [ ] Description complète avec liens
- [ ] Tags appropriés
- [ ] Partage GitHub (README, docs)
- [ ] Partage réseaux sociaux

---

## 🎯 Priorisation

### Phase 1 (Priorité Haute)
1. Wake Up
2. Greeting
3. Conversation LLM
4. Vision Tracking
5. Architecture Backend Unifié

### Phase 2 (Priorité Moyenne)
6. Emotional Response
7. Follow Face
8. Follow Object
9. Conformité SDK
10. Tests et Coverage

### Phase 3 (Priorité Basse)
11-21. Autres comportements avancés

---

## 📊 Métriques Succès

- **Vues** : Objectif 100+ par vidéo
- **Engagement** : Like/comment ratio > 5%
- **Rétention** : > 70% visionnage complet
- **Trafic GitHub** : +20% après publication

---

## 🔗 Ressources

- **Outils recommandés** : OBS Studio, DaVinci Resolve, HandBrake
- **Stock musique** : YouTube Audio Library, Free Music Archive
- **Thumbnails** : Canva, Figma
- **Hébergement** : YouTube, GitHub Releases

---

**Date de création** : 21 Novembre 2025  
**Dernière mise à jour** : 26 Novembre 2025

