"""Tests de non-regression sécurité pour HFAppInstaller."""

from pathlib import Path

import pytest

from bbia_sim.daemon.app.hf_app_installer import HFAppInstaller


@pytest.mark.asyncio
async def test_install_rejects_traversal_name(tmp_path: Path) -> None:
    installer = HFAppInstaller(apps_dir=tmp_path)
    with pytest.raises(ValueError):
        await installer.install_app("user/space", "../evil")


@pytest.mark.asyncio
@pytest.mark.parametrize(
    "bad_id",
    [
        "",
        "noslash",
        "../evil/space",
        "user/../evil",
        "user/space.git;rm",
        "https://evil.example/x",
        "user/space with space",
        "user/space\n",
        "user@host/space",
    ],
)
async def test_install_rejects_injected_space_id(tmp_path: Path, bad_id: str) -> None:
    installer = HFAppInstaller(apps_dir=tmp_path)
    with pytest.raises(ValueError, match="HF Space ID invalide"):
        await installer.install_app(bad_id)


def test_is_installed_rejects_invalid_name(tmp_path: Path) -> None:
    installer = HFAppInstaller(apps_dir=tmp_path)
    assert installer.is_installed("../evil") is False
    assert installer.is_installed("user/../evil") is False


@pytest.mark.asyncio
async def test_uninstall_rejects_invalid_name(tmp_path: Path) -> None:
    installer = HFAppInstaller(apps_dir=tmp_path)
    with pytest.raises(ValueError):
        await installer.uninstall_app("../../etc/passwd")


def test_safe_app_path_stays_under_base(tmp_path: Path) -> None:
    installer = HFAppInstaller(apps_dir=tmp_path)
    path = installer._safe_app_path("demo-app")
    assert path.parent == installer.apps_dir
    assert path.name == "demo-app"
    with pytest.raises(ValueError):
        installer._safe_app_path("a/b")
