# 🤖 RobotAPI Contract v1.1.x

## 📋 **CONTRAT D'INTERFACE UNIFIÉE SIM/ROBOT**

**Version** : 1.1.x (gelée)
**Date** : Octobre 2025
**Statut** : STABLE - Aucune modification breaking autorisée

---

## 🎯 **OBJECTIF**

Interface unifiée pour contrôler le robot Reachy Mini, que ce soit en simulation (MuJoCo) ou sur le robot réel. Même API, même comportement, même sécurité.

---

## 📐 **SIGNATURES ET TYPES**

### **Classe de base : `RobotAPI`**

```python
from abc import ABC, abstractmethod
from typing import Any, Optional

class RobotAPI(ABC):
    """Interface unifiée pour contrôler le robot (Sim ou Réel)."""

    def __init__(self):
        self.is_connected: bool = False
        self.current_emotion: str = "neutral"
        self.emotion_intensity: float = 0.5
        self.joint_limits: dict[str, tuple[float, float]] = {}
        self.forbidden_joints: set[str] = {
            "left_antenna", "right_antenna",
            "passive_1", "passive_2", "passive_3",
            "passive_4", "passive_5", "passive_6", "passive_7"
        }
        self.safe_amplitude_limit: float = 0.3  # rad
```

### **Méthodes abstraites (à implémenter)**

```python
@abstractmethod
def connect(self) -> bool:
    """Connecte au robot/simulateur.

    Returns:
        bool: True si connexion réussie, False sinon
    """

@abstractmethod
def disconnect(self) -> bool:
    """Déconnecte du robot/simulateur.

    Returns:
        bool: True si déconnexion réussie, False sinon
    """

@abstractmethod
def get_available_joints(self) -> list[str]:
    """Retourne la liste des joints disponibles.

    Returns:
        list[str]: Liste des noms de joints disponibles
    """

@abstractmethod
def set_joint_pos(self, joint_name: str, position: float) -> bool:
    """Définit la position d'un joint.

    Args:
        joint_name: Nom du joint (ex: "yaw_body")
        position: Position en radians

    Returns:
        bool: True si position définie, False sinon
    """

@abstractmethod
def get_joint_pos(self, joint_name: str) -> Optional[float]:
    """Récupère la position actuelle d'un joint.

    Args:
        joint_name: Nom du joint

    Returns:
        Optional[float]: Position en radians, None si erreur
    """

@abstractmethod
def step(self) -> bool:
    """Effectue un pas de simulation.

    Returns:
        bool: True si step réussi, False sinon
    """
```

### **Méthodes concrètes (implémentées)**

```python
def set_emotion(self, emotion: str, intensity: float = 0.5) -> bool:
    """Définit l'émotion du robot.

    Args:
        emotion: Émotion valide (voir VALID_EMOTIONS)
        intensity: Intensité 0.0-1.0 (clampée automatiquement)

    Returns:
        bool: True si émotion définie, False sinon
    """

def look_at(self, target_x: float, target_y: float, target_z: float = 0.0) -> bool:
    """Fait regarder le robot vers une cible.

    Args:
        target_x: Position X de la cible (mètres)
        target_y: Position Y de la cible (mètres)
        target_z: Position Z de la cible (mètres, défaut: 0.0)

    Returns:
        bool: True si commande envoyée, False sinon
    """

def run_behavior(self, behavior: str, intensity: float = 0.8) -> bool:
    """Exécute un comportement prédéfini.

    Args:
        behavior: Comportement valide (voir VALID_BEHAVIORS)
        intensity: Intensité 0.0-1.0 (clampée automatiquement)

    Returns:
        bool: True si comportement lancé, False sinon
    """

def get_telemetry(self) -> dict[str, Any]:
    """Récupère les métriques de performance.

    Returns:
        dict: Métriques (steps/s, avg_step_time, max_drift, etc.)
    """

def is_connected(self) -> bool:
    """Vérifie si le robot est connecté.

    Returns:
        bool: True si connecté, False sinon
    """

def is_viewer_running(self) -> bool:
    """Vérifie si le viewer graphique est actif.

    Returns:
        bool: True si viewer actif, False sinon
    """

def get_status(self) -> dict[str, Any]:
    """Récupère le statut complet du robot.

    Returns:
        dict: Statut (connected, emotion, joints, etc.)
    """
```

---

## 🚨 **ERREURS ET EXCEPTIONS**

### **Erreurs levées**

```python
# Aucune exception custom levée - utilisation de bool + logging
# Toutes les erreurs sont loggées et retournent False/None
```

### **Codes d'erreur (via logging)**

| Code | Description | Action |
|------|-------------|--------|
| `ROBOT_NOT_CONNECTED` | Robot non connecté | Retourner False |
| `INVALID_JOINT` | Joint introuvable/interdit | Retourner False |
| `INVALID_EMOTION` | Émotion non supportée | Retourner False |
| `INVALID_BEHAVIOR` | Comportement non supporté | Retourner False |
| `AMPLITUDE_CLAMPED` | Amplitude trop élevée | Clamper + Warning |
| `INTENSITY_CLAMPED` | Intensité hors limites | Clamper + Warning |

---

## 🔒 **LIMITES ET CONTRAINTES**

### **Amplitude des joints**
- **Limite absolue** : ±0.3 rad (18°)
- **Clamp automatique** : Si dépassement → Warning + clamp
- **Joints interdits** : Antennes et passifs bloqués

### **Émotions valides**
```python
VALID_EMOTIONS = {
    "neutral", "happy", "sad", "angry", "surprised",
    "confused", "determined", "nostalgic", "proud"
}
```

### **Comportements valides**
```python
VALID_BEHAVIORS = {
    "wake_up", "greeting", "emotional_response",
    "vision_tracking", "conversation", "antenna_animation", "hide"
}
```

### **Intensité**
- **Limite** : 0.0 ≤ intensity ≤ 1.0
- **Clamp automatique** : Si dépassement → Warning + clamp

---

## 🏭 **FACTORY PATTERN**

### **RobotFactory**

```python
class RobotFactory:
    @staticmethod
    def create_backend(backend_type: str = "mujoco", **kwargs) -> Optional[RobotAPI]:
        """Crée une instance de backend RobotAPI.

        Args:
            backend_type: "mujoco" ou "reachy"
            **kwargs: Arguments spécifiques au backend

        Returns:
            Optional[RobotAPI]: Instance du backend, None si erreur
        """
```

### **Backends supportés**

| Backend | Classe | Arguments |
|---------|--------|-----------|
| `"mujoco"` | `MuJoCoBackend` | `model_path: str` |
| `"reachy"` | `ReachyBackend` | `robot_ip: str`, `robot_port: int` |

---

## 🧪 **TESTS ET VALIDATION**

### **Tests obligatoires**
- ✅ Connexion/déconnexion
- ✅ Validation des joints
- ✅ Clamp des amplitudes
- ✅ Validation des émotions/comportements
- ✅ Performance (steps/s)
- ✅ Consistance entre backends

### **Tolérances**
- **Amplitude** : ±0.001 rad
- **Performance** : > 10 steps/s
- **Latence** : < 100ms par commande

---

## 📈 **MÉTRIQUES ET TÉLÉMÉTRIE**

### **Métriques collectées**
```python
{
    "steps_per_second": float,
    "average_step_time": float,
    "max_drift": float,
    "total_steps": int,
    "uptime": float,
    "current_emotion": str,
    "emotion_intensity": float,
    "active_joints": list[str]
}
```

### **Export**
- **Format** : CSV dans `artifacts/`
- **Fréquence** : À chaque démo
- **Rétention** : 30 jours

---

## 🔄 **VERSIONING ET COMPATIBILITÉ**

### **Versioning**
- **Major** : Breaking changes (jamais prévu)
- **Minor** : Nouvelles fonctionnalités
- **Patch** : Corrections de bugs

### **Compatibilité**
- **Python** : 3.10+
- **Backward** : Garantie pour v1.1.x
- **Forward** : Non garantie

---

## ⚠️ **RÈGLES DE NON-RÉGRESSION**

1. **Seed global** : Toujours fixé à 42
2. **Amplitude** : Jamais > 0.3 rad
3. **Joints interdits** : Jamais d'antennes/passifs
4. **Tests** : 1 test headless par démo obligatoire
5. **Performance** : < 5s pour tous les tests smoke

---

## 📝 **EXEMPLES D'USAGE**

### **Usage basique**
```python
from bbia_sim.robot_factory import RobotFactory

# Créer le backend
robot = RobotFactory.create_backend("mujoco")

# Connecter
if robot.connect():
    # Définir une émotion
    robot.set_emotion("happy", 0.8)

    # Animer un joint
    robot.set_joint_pos("yaw_body", 0.2)
    robot.step()

    # Déconnecter
    robot.disconnect()
```

### **Usage avec validation**
```python
# Vérifier les joints disponibles
joints = robot.get_available_joints()
if "yaw_body" in joints:
    robot.set_joint_pos("yaw_body", 0.1)

# Vérifier la connexion
if robot.is_connected():
    status = robot.get_status()
    print(f"Émotion: {status['current_emotion']}")
```

---

## 🎯 **CONTRAT GELÉ**

**Ce contrat est GELÉ en v1.1.x. Aucune modification breaking n'est autorisée.**

Pour les modifications futures :
1. Créer une nouvelle version (v1.2.x)
2. Maintenir la compatibilité backward
3. Mettre à jour ce contrat
4. Valider avec tous les tests

---

**Signé** : BBIA-REACHY-SIM Team
**Date** : Octobre 2025
**Statut** : ✅ VALIDÉ ET GELÉ
