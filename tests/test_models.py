"""Tests unitaires pour les modèles Pydantic."""

import pytest
from pydantic import ValidationError

from src.bbia_sim.daemon.models import (
    GripperControl,
    HeadControl,
    JointPosition,
    MotionCommand,
    Pose,
    TelemetryMessage,
)


class TestPose:
    """Tests pour le modèle Pose."""

    def test_valid_pose(self):
        """Test pose valide."""
        pose = Pose(x=0.5, y=0.5, z=1.0, roll=0.1, pitch=0.2, yaw=0.3)
        assert pose.x == 0.5
        assert pose.y == 0.5
        assert pose.z == 1.0
        assert pose.roll == 0.1
        assert pose.pitch == 0.2
        assert pose.yaw == 0.3

    def test_pose_defaults(self):
        """Test valeurs par défaut."""
        pose = Pose(x=0.5, y=0.5, z=1.0)
        assert pose.roll == 0.0
        assert pose.pitch == 0.0
        assert pose.yaw == 0.0

    def test_pose_bounds_validation(self):
        """Test validation des bornes."""
        # Test limites X/Y
        with pytest.raises(ValidationError):
            Pose(x=1.1, y=0.5, z=1.0)  # x > 1

        with pytest.raises(ValidationError):
            Pose(x=0.5, y=-1.1, z=1.0)  # y < -1

        # Test limite Z
        with pytest.raises(ValidationError):
            Pose(x=0.5, y=0.5, z=-0.1)  # z < 0

        with pytest.raises(ValidationError):
            Pose(x=0.5, y=0.5, z=2.1)  # z > 2

        # Test limites rotation
        with pytest.raises(ValidationError):
            Pose(x=0.5, y=0.5, z=1.0, roll=3.2)  # roll > π

        with pytest.raises(ValidationError):
            Pose(x=0.5, y=0.5, z=1.0, pitch=-3.2)  # pitch < -π


class TestJointPosition:
    """Tests pour le modèle JointPosition."""

    def test_valid_joint_position(self):
        """Test position articulation valide."""
        joint = JointPosition(joint_name="yaw_body", position=0.5)
        assert joint.joint_name == "yaw_body"
        assert joint.position == 0.5

    def test_joint_name_validation(self):
        """Test validation nom articulation."""
        # Nom trop court
        with pytest.raises(ValidationError):
            JointPosition(joint_name="", position=0.5)

        # Nom trop long
        with pytest.raises(ValidationError):
            JointPosition(joint_name="a" * 51, position=0.5)

    def test_joint_position_bounds(self):
        """Test validation bornes position."""
        # Position valide
        joint = JointPosition(joint_name="yaw_body", position=3.14)
        assert joint.position == 3.14

        # Position hors limites
        with pytest.raises(ValidationError):
            JointPosition(joint_name="yaw_body", position=3.15)  # > π

        with pytest.raises(ValidationError):
            JointPosition(joint_name="yaw_body", position=-3.15)  # < -π

        # Articulation non autorisée
        with pytest.raises(ValidationError):
            JointPosition(joint_name="invalid_joint", position=0.5)


class TestMotionCommand:
    """Tests pour le modèle MotionCommand."""

    def test_valid_motion_command(self):
        """Test commande mouvement valide."""
        cmd = MotionCommand(command="move_to", parameters={"x": 0.5, "y": 0.5})
        assert cmd.command == "move_to"
        assert cmd.parameters == {"x": 0.5, "y": 0.5}

    def test_motion_command_defaults(self):
        """Test valeurs par défaut."""
        cmd = MotionCommand(command="test")
        assert cmd.command == "test"
        assert cmd.parameters == {}

    def test_motion_command_validation(self):
        """Test validation commande."""
        # Commande vide
        with pytest.raises(ValidationError):
            MotionCommand(command="")

        # Commande trop longue
        with pytest.raises(ValidationError):
            MotionCommand(command="a" * 101)

        # Trop de paramètres
        large_params = {f"key{i}": f"value{i}" for i in range(11)}
        with pytest.raises(ValidationError):
            MotionCommand(command="test", parameters=large_params)


class TestGripperControl:
    """Tests pour le modèle GripperControl."""

    def test_valid_gripper_control(self):
        """Test contrôle pince valide."""
        gripper = GripperControl(side="left", action="open")
        assert gripper.side == "left"
        assert gripper.action == "open"

    def test_gripper_side_validation(self):
        """Test validation côté pince."""
        # Côtés valides
        assert GripperControl(side="left", action="open").side == "left"
        assert GripperControl(side="right", action="close").side == "right"

        # Côté invalide
        with pytest.raises(ValidationError):
            GripperControl(side="center", action="open")

    def test_gripper_action_validation(self):
        """Test validation action pince."""
        # Actions valides
        assert GripperControl(side="left", action="open").action == "open"
        assert GripperControl(side="left", action="close").action == "close"
        assert GripperControl(side="left", action="grip").action == "grip"

        # Action invalide
        with pytest.raises(ValidationError):
            GripperControl(side="left", action="invalid")


class TestHeadControl:
    """Tests pour le modèle HeadControl."""

    def test_valid_head_control(self):
        """Test contrôle tête valide."""
        head = HeadControl(yaw=0.5, pitch=0.2)
        assert head.yaw == 0.5
        assert head.pitch == 0.2

    def test_head_control_required_fields(self):
        """Test champs requis."""
        # Les deux champs sont requis
        with pytest.raises(ValidationError):
            HeadControl()

        with pytest.raises(ValidationError):
            HeadControl(yaw=0.5)  # pitch manquant

        with pytest.raises(ValidationError):
            HeadControl(pitch=0.2)  # yaw manquant

    def test_head_control_bounds(self):
        """Test validation bornes tête."""
        # Valeurs valides
        head = HeadControl(yaw=1.57, pitch=0.5)
        assert head.yaw == 1.57
        assert head.pitch == 0.5

        # Yaw hors limites
        with pytest.raises(ValidationError):
            HeadControl(yaw=1.58)  # > π/2

        with pytest.raises(ValidationError):
            HeadControl(yaw=-1.58)  # < -π/2

        # Pitch hors limites
        with pytest.raises(ValidationError):
            HeadControl(pitch=0.51)  # > 0.5

        with pytest.raises(ValidationError):
            HeadControl(pitch=-0.51)  # < -0.5


class TestTelemetryMessage:
    """Tests pour le modèle TelemetryMessage."""

    def test_valid_telemetry_message(self):
        """Test message télémétrie valide."""
        msg = TelemetryMessage(type="telemetry", data={"battery": 85.0})
        assert msg.type == "telemetry"
        assert msg.data == {"battery": 85.0}

    def test_telemetry_message_defaults(self):
        """Test valeurs par défaut."""
        msg = TelemetryMessage(type="ping")
        assert msg.type == "ping"
        assert msg.data == {}

    def test_telemetry_type_validation(self):
        """Test validation type message."""
        # Types valides
        valid_types = ["ping", "pong", "status", "telemetry"]
        for msg_type in valid_types:
            msg = TelemetryMessage(type=msg_type)
            assert msg.type == msg_type

        # Type invalide
        with pytest.raises(ValidationError):
            TelemetryMessage(type="invalid")

    def test_telemetry_data_size_validation(self):
        """Test validation taille données."""
        # Données normales
        msg = TelemetryMessage(type="telemetry", data={"key": "value"})
        assert msg.data == {"key": "value"}

        # Données trop volumineuses
        large_data = {f"key{i}": f"value{i}" for i in range(51)}
        with pytest.raises(ValidationError):
            TelemetryMessage(type="telemetry", data=large_data)
