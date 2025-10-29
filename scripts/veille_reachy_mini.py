#!/usr/bin/env python3
"""
Veille Reachy Mini: recherche de dépôts/profils proches et scoring de similarité.

Fonctions:
- Recherche GitHub (API v3) avec liste de mots-clés configurés
- Recherche Hugging Face Spaces (si huggingface_hub dispo)
- Scoring: API réseau, CI/coverage, dashboard, simu/MuJoCo, émotions/cognition
- Sortie: CSV append dans log/veille_reachy_mini.csv (date, source, item, score, détails)

Variables d'env:
- GH_TOKEN (optionnel): jeton GitHub pour relever les limites de rate limiting

Dépendances optionnelles:
- requests
- huggingface_hub (facultatif)
"""

from __future__ import annotations

import csv
import datetime as dt
import json
import os
import re
import sys
from dataclasses import dataclass
from typing import Any

try:
    import requests  # type: ignore
except Exception:  # pragma: no cover
    print(
        "[veille] Module 'requests' manquant. Installez requirements/requirements-veille.txt.",
        file=sys.stderr,
    )
    raise

try:
    from huggingface_hub import list_spaces  # type: ignore
except Exception:
    list_spaces = None  # type: ignore[assignment]


DEFAULT_KEYWORDS: list[str] = [
    "Reachy Mini",
    "ReachyMini",
    "Pollen Robotics Reachy Mini",
    "SDK Reachy Mini",
    "MuJoCo Reachy Mini",
    "WebSocket Reachy Mini",
    "REST API Reachy Mini",
    "Dashboard Reachy Mini",
    "Emotion engine Reachy Mini",
    "Cognitive architecture Reachy Mini",
    "LLM Reachy Mini",
    "TTS Reachy Mini",
    "STT Reachy Mini",
    "ONNX Reachy Mini",
    "Unity Reachy Mini",
    "ROS2 Reachy Mini",
    "CI coverage Reachy Mini",
]

# Expressions pour filtrage Reachy Mini strict
REACHY_MINI_PATTERNS = [
    re.compile(r"\breachy\s*mini\b", re.IGNORECASE),
    re.compile(r"\breachymini\b", re.IGNORECASE),
]
NEGATIVE_REACHY_PATTERNS = [
    re.compile(r"\breachy\s*2\b", re.IGNORECASE),
    re.compile(r"\breachy2\b", re.IGNORECASE),
]


@dataclass
class Candidate:
    source: str
    identifier: str  # URL ou repo full_name
    title: str
    description: str
    score: int
    details: dict[str, bool]


def ensure_log_csv(path: str) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    if not os.path.exists(path):
        with open(path, "w", newline="", encoding="utf-8") as f:
            writer = csv.writer(f)
            writer.writerow(
                [
                    "timestamp",
                    "source",
                    "identifier",
                    "title",
                    "score",
                    "details_json",
                    "description",
                ]
            )


def github_search_repos(
    keywords: list[str], gh_token: str | None, since_date: str | None = None
) -> list[dict]:
    headers = {"Accept": "application/vnd.github+json"}
    if gh_token:
        headers["Authorization"] = f"Bearer {gh_token}"

    results: list[dict] = []
    session = requests.Session()
    session.headers.update(headers)

    # Construire une requête OR limitée pour éviter URL trop longues
    # On itère par mot-clé pour rester simple et fiable
    stop_on_limit = os.environ.get("VEILLE_STOP_ON_RATE_LIMIT", "true").lower() in {
        "1",
        "true",
        "yes",
    }
    for kw in keywords:
        q = f"{kw} in:name,description,readme"
        if since_date:
            q += f" pushed:>={since_date}"
        url = "https://api.github.com/search/repositories"
        params = {"q": q, "sort": "updated", "order": "desc", "per_page": 10}
        try:
            resp = session.get(url, params=params, timeout=20)
            if resp.status_code == 200:
                payload = resp.json()
                items = payload.get("items", [])
                for it in items:
                    results.append(it)
            elif resp.status_code == 403 and "rate limit" in resp.text.lower():
                if os.environ.get("VEILLE_ONLY_CHANGES", "false").lower() not in {
                    "1",
                    "true",
                    "yes",
                }:
                    print(
                        "[veille][github] Rate limited — réduisez la fréquence ou définissez GH_TOKEN",
                        file=sys.stderr,
                    )
                if stop_on_limit:
                    break
            else:
                print(
                    f"[veille][github] HTTP {resp.status_code} pour '{kw}': {resp.text[:200]}",
                    file=sys.stderr,
                )
        except Exception as exc:  # pragma: no cover
            print(f"[veille][github] Erreur pour '{kw}': {exc}", file=sys.stderr)
    # Dédupliquer par full_name
    dedup: dict[str, dict] = {}
    for it in results:
        dedup[it.get("full_name", it.get("html_url", ""))] = it
    return list(dedup.values())


def github_fetch_repo_signals(repo: dict, gh_token: str | None) -> dict[str, bool]:
    headers = {"Accept": "application/vnd.github+json"}
    if gh_token:
        headers["Authorization"] = f"Bearer {gh_token}"
    session = requests.Session()
    session.headers.update(headers)

    full_name = repo.get("full_name")
    api_base = f"https://api.github.com/repos/{full_name}"

    has_ci = False
    has_dashboard = False
    has_api = False
    has_ws = False
    has_mujoco = False
    has_emotion = False
    has_tests = False
    has_docker = False
    topics: list[str] = []

    # CI: présence d'un workflow
    try:
        wf = session.get(f"{api_base}/actions/workflows", timeout=20)
        if wf.status_code == 200 and wf.json().get("total_count", 0) > 0:
            has_ci = True
    except Exception:
        pass

    # Topics (peuvent inclure reachy-mini)
    try:
        t = session.get(
            f"{api_base}/topics",
            headers={**headers, "Accept": "application/vnd.github.mercy-preview+json"},
            timeout=20,
        )
        if t.status_code == 200:
            topics = t.json().get("names", []) or []
    except Exception:
        pass

    # Lire README brut
    readme = session.get(f"{api_base}/readme", timeout=20)
    readme_text = ""
    if readme.status_code == 200:
        # La route renvoie le contenu en base64 si Accept non raw. Utilisons raw pour simplifier.
        raw_url = readme.json().get("download_url")
        if raw_url:
            try:
                raw = session.get(raw_url, timeout=20)
                if raw.status_code == 200:
                    readme_text = raw.text.lower()
            except Exception:
                pass

    txt = readme_text
    if txt:
        if re.search(r"\brest\b|\bapi\b", txt):
            has_api = True
        if "websocket" in txt:
            has_ws = True
        if "mujoco" in txt or "simulation" in txt or "simulator" in txt:
            has_mujoco = True
        if "dashboard" in txt or "web ui" in txt or "frontend" in txt:
            has_dashboard = True
        if (
            "emotion" in txt
            or "affective" in txt
            or "mood" in txt
            or "cognitive" in txt
        ):
            has_emotion = True

        # Badges et indices
        if "pytest" in txt or "unit test" in txt or "tests/" in txt:
            has_tests = True
        if "coverage" in txt or "codecov" in txt or "coveralls" in txt:
            has_ci = has_ci or True
        if "dockerfile" in txt or "docker compose" in txt or "docker-compose" in txt:
            has_docker = True

    # Heuristiques fichiers (arbre racine simplifié)
    try:
        tree = session.get(f"{api_base}/contents", timeout=20)
        if tree.status_code == 200 and isinstance(tree.json(), list):
            names = {str(x.get("name", "")).lower() for x in tree.json()}
            if any(
                n.startswith("dockerfile") or n == "docker-compose.yml" for n in names
            ):
                has_docker = True
            if ".github" in names or "tests" in names:
                has_tests = True
    except Exception:
        pass

    return {
        "ci": has_ci,
        "dashboard": has_dashboard,
        "api": has_api,
        "ws": has_ws,
        "mujoco": has_mujoco,
        "emotion": has_emotion,
        "tests": has_tests,
        "docker": has_docker,
        "topic_reachy_mini": any(
            "reachy" in t.lower() and "mini" in t.lower() for t in topics
        ),
    }


def score_from_signals(signals: dict[str, bool]) -> int:
    score = 0
    score += 1 if (signals.get("api") or signals.get("ws")) else 0
    score += 1 if signals.get("ci") else 0
    score += 1 if signals.get("dashboard") else 0
    score += 1 if signals.get("mujoco") else 0
    score += 1 if signals.get("emotion") else 0
    # Bonus légers
    score += 1 if signals.get("tests") else 0
    score += 1 if signals.get("topic_reachy_mini") else 0
    return score


def _is_reachy_mini_text(text: str) -> bool:
    if any(p.search(text) for p in NEGATIVE_REACHY_PATTERNS):
        return False
    return any(p.search(text) for p in REACHY_MINI_PATTERNS)


def search_github_candidates(
    keywords: list[str], gh_token: str | None, since_date: str | None = None
) -> list[Candidate]:
    repos = github_search_repos(keywords, gh_token, since_date=since_date)
    cands: list[Candidate] = []
    for r in repos:
        full_name = r.get("full_name", "")
        html_url = r.get("html_url", "")
        desc = r.get("description") or ""
        signals = github_fetch_repo_signals(r, gh_token)
        score = score_from_signals(signals)
        # Filtrage strict Reachy Mini
        text = f"{full_name} {desc}".lower()
        plausible = _is_reachy_mini_text(text) or signals.get(
            "topic_reachy_mini", False
        )
        identifier = html_url or full_name
        if plausible:
            cands.append(
                Candidate(
                    source="github",
                    identifier=identifier,
                    title=full_name,
                    description=desc,
                    score=score,
                    details=signals,
                )
            )
    return cands


def search_hf_candidates(keywords: list[str]) -> list[Candidate]:
    if list_spaces is None:
        return []
    cands: list[Candidate] = []
    for kw in keywords:
        try:
            for sp in list_spaces(search=kw, limit=20):  # type: ignore
                title = getattr(sp, "id", "") or getattr(sp, "title", "")
                if not title:
                    continue
                id_url = f"https://huggingface.co/spaces/{title}"
                desc = getattr(sp, "description", "") or ""
                plausible = ("reachy" in title.lower()) or ("reachy" in desc.lower())
                if not plausible:
                    continue
                # Heuristique simple sur description
                text = (f"{title} {desc}").lower()
                signals = {
                    "ci": False,  # HF n’indique pas CI
                    "dashboard": True,  # un Space est une UI
                    "api": "api" in text or "rest" in text,
                    "ws": "websocket" in text,
                    "mujoco": "mujoco" in text or "simulation" in text,
                    "emotion": "emotion" in text or "cognit" in text or "mood" in text,
                }
                score = score_from_signals(signals)
                cands.append(
                    Candidate(
                        source="huggingface",
                        identifier=id_url,
                        title=title,
                        description=desc,
                        score=score,
                        details=signals,
                    )
                )
        except Exception as exc:  # pragma: no cover
            print(f"[veille][hf] Erreur pour '{kw}': {exc}", file=sys.stderr)
    # Dédupliquer par identifier
    uniq: dict[str, Candidate] = {c.identifier: c for c in cands}
    return list(uniq.values())


def write_csv_rows(path: str, candidates: list[Candidate]) -> None:
    ensure_log_csv(path)
    now = dt.datetime.utcnow().isoformat()
    with open(path, "a", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        for c in candidates:
            writer.writerow(
                [
                    now,
                    c.source,
                    c.identifier,
                    c.title,
                    c.score,
                    json.dumps(c.details, ensure_ascii=False),
                    c.description,
                ]
            )


def trim_csv_rows(path: str, max_entries: int) -> None:
    if max_entries <= 0 or not os.path.exists(path):
        return
    try:
        with open(path, encoding="utf-8") as f:
            lines = f.readlines()
        if not lines:
            return
        header = lines[0]
        data = lines[1:]
        if len(data) <= max_entries:
            return
        trimmed = data[-max_entries:]
        with open(path, "w", encoding="utf-8") as f:
            f.write(header)
            f.writelines(trimmed)
    except Exception:
        pass


def write_json(path: str, candidates: list[Candidate]) -> None:
    os.makedirs(os.path.dirname(path), exist_ok=True)
    now = dt.datetime.utcnow().isoformat()
    payload = {
        "timestamp": now,
        "candidates": [
            {
                "source": c.source,
                "identifier": c.identifier,
                "title": c.title,
                "score": c.score,
                "details": c.details,
                "description": c.description,
            }
            for c in candidates
        ],
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)


def load_previous_json(path: str) -> dict[str, dict[str, Any]]:
    if not os.path.exists(path):
        return {}
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        prev: dict[str, dict[str, Any]] = {}
        for c in data.get("candidates", []):
            prev[str(c.get("identifier"))] = c
        return prev
    except Exception:
        return {}


def get_previous_timestamp_iso(path: str) -> str | None:
    if not os.path.exists(path):
        return None
    try:
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        ts = data.get("timestamp")
        if isinstance(ts, str) and ts:
            try:
                dt_obj = dt.datetime.fromisoformat(ts.replace("Z", "+00:00"))
            except Exception:
                return None
            return dt_obj.strftime("%Y-%m-%d")
    except Exception:
        return None
    return None


def write_diff_json(
    path: str, prev: dict[str, dict[str, Any]], curr: list[Candidate]
) -> None:
    now = dt.datetime.utcnow().isoformat()
    curr_map: dict[str, dict[str, Any]] = {
        c.identifier: {
            "source": c.source,
            "identifier": c.identifier,
            "title": c.title,
            "score": c.score,
            "details": c.details,
            "description": c.description,
        }
        for c in curr
    }

    prev_keys = set(prev.keys())
    curr_keys = set(curr_map.keys())
    new_ids = sorted(curr_keys - prev_keys)
    removed_ids = sorted(prev_keys - curr_keys)

    changed: list[dict[str, Any]] = []
    for i in prev_keys & curr_keys:
        prev_score = int(prev[i].get("score", 0))
        curr_score = int(curr_map[i].get("score", 0))
        if prev_score != curr_score:
            changed.append(
                {
                    "identifier": i,
                    "prev_score": prev_score,
                    "curr_score": curr_score,
                }
            )

    payload = {
        "timestamp": now,
        "new": [curr_map[i] for i in new_ids],
        "removed": [prev[i] for i in removed_ids],
        "changed_scores": changed,
    }
    with open(path, "w", encoding="utf-8") as f:
        json.dump(payload, f, ensure_ascii=False, indent=2)


def main(argv: list[str]) -> int:
    gh_token = os.environ.get("GH_TOKEN")
    # Permettre d’écraser la liste via un fichier si souhaité
    keywords = DEFAULT_KEYWORDS[:]  # shallow copy

    out_csv = os.path.join("log", "veille_reachy_mini.csv")
    out_json = os.path.join("log", "veille_reachy_mini.json")
    out_diff = os.path.join("log", "veille_diff.json")
    off_json = os.path.join("log", "reachy_official_status.json")
    off_diff = os.path.join("log", "reachy_official_diff.json")

    # Options via variables d'environnement
    min_score = int(os.environ.get("VEILLE_MIN_SCORE", "0") or 0)
    exclude_official = os.environ.get("VEILLE_EXCLUDE_OFFICIAL", "false").lower() in {
        "1",
        "true",
        "yes",
    }
    watch_official = os.environ.get("VEILLE_WATCH_OFFICIAL", "true").lower() in {
        "1",
        "true",
        "yes",
    }
    official_news_file = os.environ.get("OFFICIAL_NEWS_FILE")
    # Anti rate-limit/performances
    try:
        sleep_between = float(os.environ.get("VEILLE_SLEEP_BETWEEN", "0.0") or 0.0)
    except Exception:
        sleep_between = 0.0

    # Optionnel: petite pause entre sources pour ménager les quotas
    prev_overall_ts = get_previous_timestamp_iso(out_json)
    gh_candidates = search_github_candidates(
        keywords, gh_token, since_date=prev_overall_ts
    )
    if sleep_between > 0:
        try:
            import time as _t

            _t.sleep(sleep_between)
        except Exception:
            pass
    hf_candidates = search_hf_candidates(keywords)

    # Fusion
    all_candidates = gh_candidates + hf_candidates

    # Exclusions optionnelles
    if exclude_official:

        def _is_official(c: Candidate) -> bool:
            return c.identifier.startswith(
                "https://github.com/pollen-robotics/"
            ) or c.identifier.startswith(
                "https://huggingface.co/spaces/pollen-robotics/"
            )

        all_candidates = [c for c in all_candidates if not _is_official(c)]

    # Seuil de score
    if min_score > 0:
        all_candidates = [c for c in all_candidates if c.score >= min_score]

    # Tri final
    all_candidates.sort(key=lambda c: c.score, reverse=True)

    # Diff avant réécriture
    prev_map = load_previous_json(out_json)

    write_csv_rows(out_csv, all_candidates)
    # Limiter la taille du CSV aux N dernières exécutions (par défaut 10)
    try:
        max_entries = int(os.environ.get("VEILLE_MAX_ENTRIES", "5") or 5)
    except Exception:
        max_entries = 5
    trim_csv_rows(out_csv, max_entries=max_entries)
    write_json(out_json, all_candidates)
    write_diff_json(out_diff, prev_map, all_candidates)

    # Résumé console avec option seulement en cas de changements
    only_changes = os.environ.get("VEILLE_ONLY_CHANGES", "false").lower() in {
        "1",
        "true",
        "yes",
    }
    changes_new = changes_removed = changes_scores = 0
    try:
        with open(out_diff, encoding="utf-8") as f:
            diff = json.load(f)
        changes_new = len(diff.get("new", []))
        changes_removed = len(diff.get("removed", []))
        changes_scores = len(diff.get("changed_scores", []))
    except Exception:
        pass
    if not only_changes:
        print(
            f"[veille] {len(all_candidates)} candidats consignés dans {out_csv} et {out_json}"
        )
        print(
            f"[veille] Nouveaux: {changes_new}, Retirés: {changes_removed}, Scores modifiés: {changes_scores}"
        )
        for c in all_candidates[:10]:
            print(f"  - [{c.score}] {c.title} -> {c.identifier}")
    else:
        if changes_new or changes_removed or changes_scores:
            print(
                f"[veille] Changements détectés — Nouveaux:{changes_new} Retirés:{changes_removed} Scores:{changes_scores}"
            )
            for c in all_candidates[:5]:
                print(f"  - [{c.score}] {c.title} -> {c.identifier}")

    # Suivi officiel pollen-robotics/reachy_mini et fichier de news
    if watch_official:
        try:
            status: dict[str, Any] = {}
            headers = {"Accept": "application/vnd.github+json"}
            if gh_token:
                headers["Authorization"] = f"Bearer {gh_token}"
            sess = requests.Session()
            sess.headers.update(headers)

            repo = sess.get(
                "https://api.github.com/repos/pollen-robotics/reachy_mini", timeout=20
            )
            if repo.status_code == 200:
                rj = repo.json()
                default_branch = rj.get("default_branch", "main")
                status.update(
                    {
                        "repo": "pollen-robotics/reachy_mini",
                        "stargazers": rj.get("stargazers_count"),
                        "open_issues": rj.get("open_issues_count"),
                        "pushed_at": rj.get("pushed_at"),
                        "default_branch": default_branch,
                    }
                )
                rel = sess.get(
                    "https://api.github.com/repos/pollen-robotics/reachy_mini/releases/latest",
                    timeout=20,
                )
                if rel.status_code == 200:
                    r = rel.json()
                    status["latest_release_tag"] = r.get("tag_name")
                    status["latest_release_published_at"] = r.get("published_at")
                commits = sess.get(
                    f"https://api.github.com/repos/pollen-robotics/reachy_mini/commits?sha={default_branch}&per_page=1",
                    timeout=20,
                )
                if (
                    commits.status_code == 200
                    and isinstance(commits.json(), list)
                    and commits.json()
                ):
                    c0 = commits.json()[0]
                    status["last_commit_sha"] = c0.get("sha")
                    status["last_commit_date"] = (
                        (c0.get("commit") or {}).get("author") or {}
                    ).get("date")

            if official_news_file and os.path.exists(official_news_file):
                try:
                    import hashlib
                    import re
                    import time

                    mtime = os.path.getmtime(official_news_file)
                    with open(official_news_file, "rb") as nf:
                        blob = nf.read()
                    # Parsing simple pour extraire chiffres/dates clefs
                    text = ""
                    try:
                        text = blob.decode("utf-8", errors="ignore")
                    except Exception:
                        text = ""
                    status["news_file_path"] = official_news_file
                    status["news_file_mtime"] = mtime
                    status["news_file_mtime_iso"] = time.strftime(
                        "%Y-%m-%dT%H:%M:%SZ", time.gmtime(mtime)
                    )
                    status["news_file_sha256"] = hashlib.sha256(blob).hexdigest()
                    # Heuristiques de parsing
                    lower = text.lower()
                    # beta_units: chercher "beta shipments" et un nombre proche
                    m_beta = re.search(
                        r"beta\s+shipments[^\d]*(\d{1,4})\s+units", lower
                    )
                    if m_beta:
                        status.setdefault("shipping", {})
                        status["shipping"]["beta_units"] = int(m_beta.group(1))
                    # plan_units_q4: "around 3,000 Reachy-mini units" (tolérer virgule)
                    m_plan = re.search(
                        r"around\s+([\d,]+)\s+reachy[-\s]?mini\s+units", lower
                    )
                    if m_plan:
                        try:
                            units = int(m_plan.group(1).replace(",", ""))
                            status.setdefault("shipping", {})
                            status["shipping"]["plan_units_q4"] = units
                        except Exception:
                            pass
                    # next update: "Next update" ... "mid-November"
                    m_next = re.search(r"next\s+update[^\n]*mid[-\s]?november", lower)
                    if m_next:
                        now = dt.datetime.utcnow()
                        year = now.year
                        # si on est déjà fin novembre/décembre, projeter à l'année suivante
                        if now.month > 11:
                            year += 1
                        status.setdefault("shipping", {})
                        status["shipping"]["next_update_date"] = f"{year}-11-15"
                    # batches after: "January–February"
                    if "january" in lower and "february" in lower:
                        now = dt.datetime.utcnow()
                        year = now.year + (1 if now.month > 2 else 0)
                        status.setdefault("shipping", {})
                        status["shipping"]["batches_after"] = [
                            f"{year}-01",
                            f"{year}-02",
                        ]
                except Exception:
                    pass

            prev_off = load_previous_json(off_json)
            os.makedirs(os.path.dirname(off_json), exist_ok=True)
            curr_off = {
                "timestamp": dt.datetime.utcnow().isoformat(),
                "candidates": [{"identifier": "official:reachy_mini", **status}],
            }
            with open(off_json, "w", encoding="utf-8") as f:
                json.dump(curr_off, f, ensure_ascii=False, indent=2)

            # Diff très simple champ-à-champ
            changed_fields: list[dict[str, Any]] = []
            old = prev_off.get("official:reachy_mini", {})
            for k, v in status.items():
                if old.get(k) != v:
                    changed_fields.append({"field": k, "prev": old.get(k), "curr": v})
            with open(off_diff, "w", encoding="utf-8") as f:
                json.dump(
                    {
                        "timestamp": dt.datetime.utcnow().isoformat(),
                        "changed_fields": changed_fields,
                    },
                    f,
                    ensure_ascii=False,
                    indent=2,
                )
            if changed_fields:
                print(
                    f"[veille] Officiel: {len(changed_fields)} changement(s) détecté(s)."
                )
        except Exception as exc:  # pragma: no cover
            print(f"[veille][official] Erreur suivi officiel: {exc}", file=sys.stderr)

    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
