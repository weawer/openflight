#!/usr/bin/env python3
"""Compare the IWR6843 self-trigger of several git refs on the same recorded captures.

    uv run python scripts/analysis/compare_trigger_builds.py REF [REF ...] [--group NAME]
    uv run python scripts/analysis/compare_trigger_builds.py 4c86e6f HEAD --group bench_2026-10-04

Each ref is checked out into a temporary git worktree; this checkout's
``openflight/iwr6843/trigger_eval.py`` is run inside it, so the ref's own
firmware C and replay code decide, on this checkout's captures. The report
counts fires by rule per capture group and ref, then lists every capture
whose (fired frame, rule) differs between refs. Nothing in the checkout is
changed; the worktrees are removed afterwards.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import tempfile
from collections import Counter
from pathlib import Path

from openflight.iwr6843.trigger_eval import differences

ROOT = Path(__file__).resolve().parents[2]
EVALUATOR = ROOT / "src" / "openflight" / "iwr6843" / "trigger_eval.py"


def _git(*args: str) -> str:
    return subprocess.run(
        ["git", "-C", str(ROOT), *args], check=True, capture_output=True, text=True
    ).stdout.strip()


def evaluate_ref(ref: str, groups: list[str] | None, scratch: Path) -> dict:
    """Run the evaluator inside a worktree of ``ref``; return its JSON."""
    sha = _git("rev-parse", "--verify", f"{ref}^{{commit}}")
    worktree = scratch / sha[:12]
    _git("worktree", "add", "--detach", str(worktree), sha)
    try:
        command = ["uv", "run", "--quiet", "python", "-I", str(EVALUATOR), "--root", str(ROOT)]
        for group in groups or ():
            command += ["--group", group]
        done = subprocess.run(
            [*command, "--json"], cwd=worktree, capture_output=True, text=True, check=False
        )
        if done.returncode != 0:
            raise RuntimeError(f"{ref}: the evaluator failed:\n{done.stderr.strip()}")
        return json.loads(done.stdout.strip().splitlines()[-1])
    finally:
        _git("worktree", "remove", "--force", str(worktree))


def report(results: dict[str, dict]) -> str:
    """Fire counts per group and ref, then every capture that differs."""
    lines = []
    refs = list(results)
    groups = dict.fromkeys(group for ref in refs for group in results[ref])
    for group in groups:
        lines.append(f"== {group}")
        for ref in refs:
            outcomes = results[ref].get(group, [])
            counts = Counter(o["rule"] or "none" for o in outcomes)
            text = ", ".join(f"{rule} {n}" for rule, n in sorted(counts.items()))
            lines.append(f"  {ref:>20}: {len(outcomes)} captures, {text}")
    rows = differences(results)
    lines.append("")
    lines.append(f"{len(rows)} capture(s) differ" + (":" if rows else "."))
    for group, name, seen in rows:
        cells = "  ".join(f"{ref}={frame},{rule}" for ref, (frame, rule) in seen.items())
        lines.append(f"  {group:18} {name:46} {cells}")
    return "\n".join(lines)


def main(argv: list[str] | None = None) -> int:
    """Evaluate every ref and print the comparison."""
    parser = argparse.ArgumentParser(description=__doc__.split("\n\n", 1)[0])
    parser.add_argument("refs", nargs="+", help="git refs to compare (branches, tags, SHAs)")
    parser.add_argument("--group", action="append", help="capture group(s); default all")
    parser.add_argument("--json", type=Path, help="also write the raw results here")
    args = parser.parse_args(argv)
    with tempfile.TemporaryDirectory(prefix="trigger-builds-") as scratch:
        results = {ref: evaluate_ref(ref, args.group, Path(scratch)) for ref in args.refs}
    _git("worktree", "prune")
    if args.json:
        args.json.write_text(json.dumps(results), encoding="utf-8")
    print(report(results))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
