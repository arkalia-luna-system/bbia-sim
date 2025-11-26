# 📤 Partage d'Apps BBIA sur Hugging Face Hub

**Guide pour créer et partager des applications BBIA-SIM sur Hugging Face Spaces**

---

## 🎯 Objectif

Créer et partager vos applications BBIA personnalisées sur Hugging Face Hub pour que la communauté puisse les utiliser.

---

## 📋 Prérequis

- Compte Hugging Face (gratuit) : https://huggingface.co/join
- Token HF Hub : https://huggingface.co/settings/tokens
- Application BBIA fonctionnelle

---

## 🚀 Workflow de Partage

### Étape 1 : Préparer votre Application

Votre application doit être dans un format compatible avec Hugging Face Spaces :

```
mon-app-bbia/
├── app.py              # Code principal de l'app
├── requirements.txt    # Dépendances Python
├── README.md           # Documentation
└── assets/             # Ressources (optionnel)
```

### Étape 2 : Template d'Application BBIA

**Template `app.py` minimal :**

```python
"""Application BBIA pour Hugging Face Spaces."""

import gradio as gr
from bbia_sim.robot_factory import RobotFactory

def create_bbia_app():
    """Crée l'interface Gradio pour l'app BBIA."""
    
    # Initialiser le robot
    robot = RobotFactory.create_backend("mujoco")  # ou "reachy_mini" pour robot réel
    robot.connect()
    
    def process_command(command: str) -> str:
        """Traite une commande utilisateur."""
        # Votre logique BBIA ici
        if "bonjour" in command.lower():
            robot.set_emotion("happy", 0.7)
            return "Bonjour ! Je suis heureux de vous rencontrer !"
        elif "au revoir" in command.lower():
            robot.set_emotion("sad", 0.5)
            return "Au revoir ! À bientôt !"
        else:
            return f"Commande reçue : {command}"
    
    # Interface Gradio
    with gr.Blocks(title="Mon App BBIA") as app:
        gr.Markdown("# 🤖 Mon Application BBIA")
        gr.Markdown("Application BBIA personnalisée pour Reachy Mini")
        
        with gr.Row():
            command_input = gr.Textbox(
                label="Commande",
                placeholder="Tapez votre commande...",
            )
            output = gr.Textbox(label="Réponse")
        
        submit_btn = gr.Button("Envoyer")
        submit_btn.click(
            fn=process_command,
            inputs=command_input,
            outputs=output,
        )
    
    return app

if __name__ == "__main__":
    app = create_bbia_app()
    app.launch()
```

**Template `requirements.txt` :**

```
bbia-sim>=1.4.0
gradio>=4.0.0
```

**Template `README.md` :**

```markdown
# Mon Application BBIA

Application BBIA personnalisée pour Reachy Mini.

## Fonctionnalités

- [Description des fonctionnalités]

## Installation

```bash
pip install -r requirements.txt
```

## Utilisation

Lancez l'application :

```bash
python app.py
```

## Auteur

[Votre nom]

## Licence

MIT
```

### Étape 3 : Créer un Space sur Hugging Face

1. Allez sur https://huggingface.co/new-space
2. Remplissez les informations :
   - **Nom** : `mon-app-bbia` (format: `username/mon-app-bbia`)
   - **SDK** : `Gradio`
   - **Visibilité** : `Public` ou `Private`
3. Cliquez sur **Create Space**

### Étape 4 : Uploader votre Code

**Option A : Via l'interface web**
1. Allez sur votre Space
2. Cliquez sur **Files and versions**
3. Upload vos fichiers (`app.py`, `requirements.txt`, `README.md`)

**Option B : Via Git**
```bash
# Cloner votre Space
git clone https://huggingface.co/spaces/username/mon-app-bbia
cd mon-app-bbia

# Copier vos fichiers
cp /chemin/vers/votre/app.py .
cp /chemin/vers/votre/requirements.txt .
cp /chemin/vers/votre/README.md .

# Commit et push
git add .
git commit -m "Initial commit: Mon app BBIA"
git push
```

### Étape 5 : Configurer le Space

Créez un fichier `README.md` à la racine du Space avec :

```yaml
---
title: Mon Application BBIA
emoji: 🤖
colorFrom: blue
colorTo: purple
sdk: gradio
sdk_version: 4.0.0
app_file: app.py
pinned: false
---
```

### Étape 6 : Partager avec la Communauté

Une fois votre Space créé et fonctionnel :

1. **Ajoutez des tags** : `reachy-mini`, `bbia`, `robot`, `ai`
2. **Partagez le lien** : `https://huggingface.co/spaces/username/mon-app-bbia`
3. **Mentionnez dans la communauté** : Discord Reachy Mini, GitHub, etc.

---

## 📚 Exemples d'Apps BBIA à Partager

### 1. Application Conversationnelle
- Chat avec BBIA
- Reconnaissance vocale
- Synthèse vocale

### 2. Application Vision
- Détection d'objets
- Reconnaissance de visages
- Suivi d'objets

### 3. Application Mouvements
- Bibliothèque de mouvements
- Chorégraphies
- Poses personnalisées

### 4. Application IA
- Assistant IA avec LLM
- Analyse de sentiment
- Génération de réponses

---

## 🔧 Intégration avec BBIA-SIM Dashboard

Une fois votre app partagée sur HF Hub, elle sera automatiquement découverte par :

- **Endpoint** : `/api/apps/list-community`
- **Dashboard** : Section "Apps Communauté (Testeurs Bêta)"
- **Installation** : Via le bouton "Install" dans le dashboard

---

## ✅ Checklist de Partage

- [ ] Application fonctionnelle localement
- [ ] Code documenté (README.md)
- [ ] Requirements.txt à jour
- [ ] Space créé sur HF Hub
- [ ] Code uploadé et testé
- [ ] Tags ajoutés (`reachy-mini`, `bbia`)
- [ ] Lien partagé avec la communauté

---

## 🆘 Support

- **Documentation BBIA** : `docs/guides/`
- **Issues GitHub** : https://github.com/arkalia-luna-system/bbia-sim/issues
- **Communauté HF** : https://huggingface.co/spaces?search=reachy-mini

---

**Document créé le :** 21 Novembre 2025  
**Dernière mise à jour :** 26 Novembre 2025  
**Dernière mise à jour :** 26 Novembre 2025

