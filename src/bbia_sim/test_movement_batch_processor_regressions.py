"""Tests de non-regression pour MovementBatchProcessor."""

import pytest

from bbia_sim.movement_batch_processor import MovementBatchProcessor


def test_constructor_validates_parameters() -> None:
    with pytest.raises(ValueError):
        MovementBatchProcessor(max_batch_size=0)
    with pytest.raises(ValueError):
        MovementBatchProcessor(batch_timeout=-0.1)


@pytest.mark.asyncio
async def test_flush_executes_pending_movements() -> None:
    processor = MovementBatchProcessor(max_batch_size=10, batch_timeout=1.0)
    calls: list[str] = []

    async def movement_a() -> None:
        calls.append("a")

    async def movement_b() -> None:
        calls.append("b")

    await processor.add_movement(movement_a, "a")
    await processor.add_movement(movement_b, "b")
    await processor.flush()

    assert sorted(calls) == ["a", "b"]
    assert processor.get_queue_size() == 0


@pytest.mark.asyncio
async def test_wait_for_idle_returns_true_after_processing() -> None:
    processor = MovementBatchProcessor(max_batch_size=2, batch_timeout=0.01)
    calls = 0

    async def movement() -> None:
        nonlocal calls
        calls += 1

    for i in range(3):
        await processor.add_movement(movement, f"move-{i}")

    idle = await processor.wait_for_idle(timeout=1.0)
    assert idle is True
    assert calls == 3
    stats = processor.get_stats()
    assert stats["queue_size"] == 0
    assert stats["task_active"] is False


@pytest.mark.asyncio
async def test_batch_continues_when_one_movement_fails() -> None:
    processor = MovementBatchProcessor(max_batch_size=3, batch_timeout=0)
    calls: list[str] = []

    async def ok_move() -> None:
        calls.append("ok")

    async def failing_move() -> None:
        calls.append("fail")
        raise RuntimeError("boom")

    await processor.add_movement(ok_move, "ok-1")
    await processor.add_movement(failing_move, "fail")
    await processor.add_movement(ok_move, "ok-2")

    assert await processor.wait_for_idle(timeout=1.0) is True
    assert calls.count("ok") == 2
    assert calls.count("fail") == 1
