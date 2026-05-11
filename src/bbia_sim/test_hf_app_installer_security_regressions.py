"""Tests de non-regression sécurité pour HFAppInstaller."""

from pathlib import Path

import pytest

from bbia_sim.daemon.app.hf_app_installer import HFAppInstaller


@pytest.mark.asyncio
async def test_install_rejects_traversal_name(tmp_path: Path) -> None:
    installer = HFAppInstaller(apps_dir=tmp_path)
    with pytest.raises(ValueError):
        await installer.install_app("user/space", "../evil")


def test_is_installed_rejects_invalid_name(tmp_path: Path) -> None:
    installer = HFAppInstaller(apps_dir=tmp_path)
    assert installer.is_installed("../evil") is False


@pytest.mark.asyncio
async def test_uninstall_rejects_invalid_name(tmp_path: Path) -> None:
    installer = HFAppInstaller(apps_dir=tmp_path)
    with pytest.raises(ValueError):
        await installer.uninstall_app("../../etc/passwd")
