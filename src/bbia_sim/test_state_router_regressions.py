"""Tests de non-regression pour le routeur state."""

from typing import Any

import pytest
from fastapi import HTTPException

from bbia_sim.daemon.app.routers.state import (
    get_full_state,
    get_present_antenna_joint_positions,
)


class _BackendNoTargetPose:
    target_head_pose: Any | None = None
    target_head_joint_positions: Any | None = None
    target_body_yaw: float | None = None
    target_antenna_joint_positions: Any | None = None

    def get_motor_control_mode(self):
        class _Mode:
            value = "enabled"

        return _Mode()

    def get_present_head_pose(self):
        import numpy as np

        return np.eye(4)

    def get_present_head_joint_positions(self):
        return None

    def get_present_body_yaw(self):
        return 0.0

    def get_present_antenna_joint_positions(self):
        import numpy as np

        return np.array([0.0, 0.0])

    def get_present_passive_joint_positions(self):
        return None


class _BackendBadAntennaLen:
    def get_present_antenna_joint_positions(self):
        import numpy as np

        return np.array([0.0])  # invalide: longueur 1


@pytest.mark.asyncio
async def test_get_full_state_target_head_pose_none_is_safe():
    backend = _BackendNoTargetPose()
    state = await get_full_state(backend=backend, with_target_head_pose=True)
    assert state.target_head_pose is None


@pytest.mark.asyncio
async def test_get_present_antenna_joint_positions_raises_http_exception():
    backend = _BackendBadAntennaLen()
    with pytest.raises(HTTPException) as exc:
        await get_present_antenna_joint_positions(backend=backend)
    assert exc.value.status_code == 500
