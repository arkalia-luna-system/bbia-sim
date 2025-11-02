#!/usr/bin/env python3
"""Validateur de traces golden pour RobotAPI
Compare une trace courante avec une référence et valide les tolérances
"""

import argparse
import json
from statistics import mean


def load(path):
    """Charge une trace JSONL."""
    with open(path, encoding="utf-8") as f:
        return [json.loads(x) for x in f]


def compare(ref, cur, tol_q=0.05, tol_rate=0.15):
    """Compare deux traces avec tolérances.

    Args:
        ref: Trace de référence
        cur: Trace courante
        tol_q: Tolérance position (rad)
        tol_rate: Tolérance cadence (%)

    Returns:
        tuple: (ok, metrics)

    """
    assert len(ref) > 10 and len(cur) > 10, "Traces trop courtes"

    # Aligner par index (simple & robuste)
    n = min(len(ref), len(cur))
    dq = [abs(ref[i]["qpos"] - cur[i]["qpos"]) for i in range(n)]
    ok_q = max(dq) <= tol_q

    # Cadence (approx): moyenne dt
    def cadence(tr):
        dt = [tr[i + 1]["t"] - tr[i]["t"] for i in range(len(tr) - 1)]
        return 1.0 / (mean(dt) or 1e-6)

    r_rate, c_rate = cadence(ref), cadence(cur)
    ok_rate = abs(c_rate - r_rate) / max(r_rate, 1e-6) <= tol_rate

    metrics = {
        "max_abs_qpos_err": max(dq),
        "ref_hz": r_rate,
        "cur_hz": c_rate,
        "rate_diff_pct": abs(c_rate - r_rate) / max(r_rate, 1e-6) * 100,
        "frames_ref": len(ref),
        "frames_cur": len(cur),
        "frames_compared": n,
    }

    return ok_q and ok_rate, metrics


def main():
    ap = argparse.ArgumentParser(description="Validateur de traces golden")
    ap.add_argument("--ref", required=True, help="Fichier de référence")
    ap.add_argument("--cur", required=True, help="Fichier courant")
    ap.add_argument(
        "--tol-q",
        type=float,
        default=0.05,
        help="Tolérance position (rad)",
    )
    ap.add_argument(
        "--tol-rate",
        type=float,
        default=0.15,
        help="Tolérance cadence (%)",
    )
    args = ap.parse_args()

    print("🔍 Validation trace:")
    print(f"   • Référence: {args.ref}")
    print(f"   • Courante: {args.cur}")
    print(f"   • Tolérance position: ±{args.tol_q} rad")
    print(f"   • Tolérance cadence: ±{args.tol_rate * 100:.1f}%")

    try:
        ref, cur = load(args.ref), load(args.cur)
        ok, metrics = compare(ref, cur, args.tol_q, args.tol_rate)

        print("\n📊 Métriques:")
        print(f"   • Erreur max position: {metrics['max_abs_qpos_err']:.4f} rad")
        print(f"   • Cadence référence: {metrics['ref_hz']:.1f} Hz")
        print(f"   • Cadence courante: {metrics['cur_hz']:.1f} Hz")
        print(f"   • Différence cadence: {metrics['rate_diff_pct']:.1f}%")
        print(f"   • Frames comparées: {metrics['frames_compared']}")

        if ok:
            print("\n✅ Validation réussie")
            return 0
        print("\n❌ Validation échouée")
        return 1

    except Exception as e:
        print(f"\n❌ Erreur validation: {e}")
        return 1


if __name__ == "__main__":
    exit(main())
