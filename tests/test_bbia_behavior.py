#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Test du module BBIA Behavior Manager
"""

import unittest
import sys
import os
import io
import contextlib
from unittest.mock import patch

# Ajouter le chemin du module parent
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

from src.bbia_sim.bbia_behavior import (
    BBIABehavior,
    WakeUpBehavior,
    GreetingBehavior,
    EmotionalResponseBehavior,
    VisionTrackingBehavior,
    ConversationBehavior,
    AntennaAnimationBehavior,
    BBIABehaviorManager,
    HideBehavior,  # Ajouté
)


class TestBBIABehavior(unittest.TestCase):
    """Tests pour la classe de base BBIABehavior"""

    def test_behavior_creation(self):
        """Test de création d'un comportement de base"""
        behavior = BBIABehavior("test", "Test behavior")
        self.assertEqual(behavior.name, "test")
        self.assertEqual(behavior.description, "Test behavior")
        self.assertFalse(behavior.is_active)
        self.assertEqual(behavior.priority, 1)

    def test_behavior_can_execute(self):
        """Test de la méthode can_execute"""
        behavior = BBIABehavior("test", "Test behavior")
        self.assertTrue(behavior.can_execute({}))

    def test_behavior_execute(self):
        """Test de la méthode execute"""
        behavior = BBIABehavior("test", "Test behavior")
        result = behavior.execute({})
        self.assertTrue(result)


class TestWakeUpBehavior(unittest.TestCase):
    """Tests pour le comportement de réveil"""

    def test_wake_up_creation(self):
        """Test de création du comportement de réveil"""
        behavior = WakeUpBehavior()
        self.assertEqual(behavior.name, "wake_up")
        self.assertEqual(behavior.priority, 10)

    def test_wake_up_execute(self):
        """Test d'exécution du comportement de réveil"""
        behavior = WakeUpBehavior()
        result = behavior.execute({})
        self.assertTrue(result)


class TestGreetingBehavior(unittest.TestCase):
    """Tests pour le comportement de salutation"""

    def test_greeting_creation(self):
        """Test de création du comportement de salutation"""
        behavior = GreetingBehavior()
        self.assertEqual(behavior.name, "greeting")
        self.assertIsInstance(behavior.greetings, list)
        self.assertGreater(len(behavior.greetings), 0)

    def test_greeting_execute(self):
        """Test d'exécution du comportement de salutation"""
        behavior = GreetingBehavior()
        result = behavior.execute({})
        self.assertTrue(result)


class TestAntennaAnimationBehavior(unittest.TestCase):
    """Tests pour le comportement d'animation des antennes"""

    def test_antenna_creation(self):
        """Test de création du comportement d'animation des antennes"""
        behavior = AntennaAnimationBehavior()
        self.assertEqual(behavior.name, "antenna_animation")
        self.assertEqual(behavior.priority, 5)

    def test_antenna_execute_with_emotion(self):
        """Test d'exécution avec une émotion spécifique"""
        behavior = AntennaAnimationBehavior()
        context = {"emotion": "happy"}
        result = behavior.execute(context)
        self.assertTrue(result)

    def test_antenna_execute_without_emotion(self):
        """Test d'exécution sans émotion spécifiée"""
        behavior = AntennaAnimationBehavior()
        result = behavior.execute({})
        self.assertTrue(result)


class TestHideBehavior(unittest.TestCase):
    """Tests pour le comportement 'se cacher' (HideBehavior)"""

    def test_hide_creation(self):
        """Test de création du comportement 'se cacher'"""
        behavior = HideBehavior()
        self.assertEqual(behavior.name, "hide")
        self.assertEqual(behavior.priority, 9)

    def test_hide_execute(self):
        """Test d'exécution du comportement 'se cacher'"""
        behavior = HideBehavior()
        result = behavior.execute({})
        self.assertTrue(result)

    def test_hide_sequence_stdout_and_voice(self):
        """Test avancé : vérifie la séquence console et la synthèse vocale pour 'se cacher'"""
        behavior = HideBehavior()
        output = io.StringIO()
        # Correction : patcher dire_texte dans le namespace du module bbia_behavior
        with patch("src.bbia_sim.bbia_behavior.dire_texte") as mock_dire_texte:
            with contextlib.redirect_stdout(output):
                result = behavior.execute({})
        # Vérification de la sortie console
        out = output.getvalue()
        self.assertIn("🙈 [BBIA] Séquence 'se cacher'...", out)
        self.assertIn("🤖 Tête qui s'abaisse lentement...", out)
        self.assertIn("📡 Antennes qui se replient devant le visage...", out)
        self.assertIn("👁️ Yeux qui se ferment (ou s'éteignent)...", out)
        self.assertIn("💤 BBIA se cache et devient silencieux.", out)
        self.assertIn("(BBIA attend discrètement...)", out)
        # Vérification de la synthèse vocale
        mock_dire_texte.assert_called_once_with("Je me cache... Chut !")
        self.assertTrue(result)


class TestBBIABehaviorManager(unittest.TestCase):
    """Tests pour le gestionnaire de comportements"""

    def setUp(self):
        """Initialisation avant chaque test"""
        self.manager = BBIABehaviorManager()

    def test_manager_creation(self):
        """Test de création du gestionnaire"""
        self.assertIsInstance(self.manager.behaviors, dict)
        self.assertGreater(len(self.manager.behaviors), 0)
        self.assertFalse(self.manager.is_running)

    def test_register_behavior(self):
        """Test d'enregistrement d'un comportement"""
        custom_behavior = BBIABehavior("custom", "Custom behavior")
        self.manager.register_behavior(custom_behavior)
        self.assertIn("custom", self.manager.behaviors)

    def test_execute_behavior(self):
        """Test d'exécution d'un comportement"""
        result = self.manager.execute_behavior("greeting")
        self.assertTrue(result)

    def test_execute_unknown_behavior(self):
        """Test d'exécution d'un comportement inconnu"""
        result = self.manager.execute_behavior("unknown")
        self.assertFalse(result)

    def test_get_available_behaviors(self):
        """Test de récupération des comportements disponibles"""
        behaviors = self.manager.get_available_behaviors()
        self.assertIsInstance(behaviors, list)
        self.assertGreater(len(behaviors), 0)

        # Vérifier la structure des données
        if behaviors:
            behavior = behaviors[0]
            self.assertIn("name", behavior)
            self.assertIn("description", behavior)
            self.assertIn("priority", behavior)
            self.assertIn("is_active", behavior)

    def test_get_behavior_stats(self):
        """Test de récupération des statistiques"""
        stats = self.manager.get_behavior_stats()
        self.assertIsInstance(stats, dict)
        self.assertIn("total_behaviors", stats)
        self.assertIn("active_behaviors", stats)
        self.assertIn("queue_size", stats)
        self.assertIn("worker_running", stats)
        self.assertIn("available_behaviors", stats)

        self.assertGreater(stats["total_behaviors"], 0)
        self.assertIsInstance(stats["available_behaviors"], list)

    def test_add_to_queue(self):
        """Test d'ajout à la queue"""
        initial_size = self.manager.behavior_queue.qsize()
        self.manager.add_to_queue("greeting")
        new_size = self.manager.behavior_queue.qsize()
        self.assertEqual(new_size, initial_size + 1)

    def test_worker_lifecycle(self):
        """Test du cycle de vie du worker"""
        # Démarrer le worker
        self.manager.start_behavior_worker()
        self.assertTrue(self.manager.is_running)

        # Arrêter le worker
        self.manager.stop_behavior_worker()
        self.assertFalse(self.manager.is_running)


def main():
    """Fonction principale pour exécuter les tests"""
    print("🧪 Tests du module BBIA Behavior Manager")
    print("=" * 50)

    # Créer la suite de tests
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()

    # Ajouter les tests
    suite.addTests(loader.loadTestsFromTestCase(TestBBIABehavior))
    suite.addTests(loader.loadTestsFromTestCase(TestWakeUpBehavior))
    suite.addTests(loader.loadTestsFromTestCase(TestGreetingBehavior))
    suite.addTests(loader.loadTestsFromTestCase(TestAntennaAnimationBehavior))
    suite.addTests(loader.loadTestsFromTestCase(TestHideBehavior))  # Ajouté
    suite.addTests(loader.loadTestsFromTestCase(TestBBIABehaviorManager))

    # Exécuter les tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    # Afficher le résumé
    print(f"\n📊 Résumé des tests :")
    print(f"   • Tests exécutés : {result.testsRun}")
    print(f"   • Échecs : {len(result.failures)}")
    print(f"   • Erreurs : {len(result.errors)}")

    if result.failures:
        print(f"\n❌ Échecs :")
        for test, traceback in result.failures:
            print(f"   • {test}: {traceback}")

    if result.errors:
        print(f"\n❌ Erreurs :")
        for test, traceback in result.errors:
            print(f"   • {test}: {traceback}")

    if result.wasSuccessful():
        print(f"\n✅ Tous les tests ont réussi !")
    else:
        print(f"\n❌ Certains tests ont échoué.")

    return result.wasSuccessful()


if __name__ == "__main__":
    success = main()
    sys.exit(0 if success else 1)
