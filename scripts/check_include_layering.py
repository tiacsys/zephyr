#!/usr/bin/env python3
#
# Copyright (c) 2026 The Zephyr Project Contributors
#
# SPDX-License-Identifier: Apache-2.0

'''Check that Zephyr source files do not transitively #include headers from
layers they are not supposed to depend on (e.g. kernel/*.c reaching vendor
HAL headers).

Zephyr's headers form a single, unguarded include graph: anything that
#include's zephyr/kernel.h can end up transitively pulling in the SoC HAL of
whatever board happens to be selected, because of chains such as

    kernel.h -> kernel_includes.h -> arch/cpu.h -> arch/arm/arch.h ->
    arm_mpu.h -> arm_mpu_v7m.h -> cmsis_core.h -> soc.h -> stm32g4xx.h -> HAL

This script makes such cross-layer reachability measurable and enforceable:

  1. It re-runs every (or every relevant) translation unit in a build's
     compile_commands.json with `-H -fsyntax-only` and parses GCC's
     dot-prefixed include hierarchy into a direct-#include edge graph.
  2. It loads a small YAML rules file describing, per source scope
     (e.g. "kernel/**"), which header patterns must never be transitively
     reachable (e.g. "modules/hal/**").
  3. For every violation it reports the offending translation unit, the
     unreachable-in-principle header, and one concrete (shortest) include
     chain connecting the two -- the actionable part of the report.
  4. It supports a baseline workflow (like a "known issues" allowlist) so
     that a large legacy violation surface can be locked down and only
     *new* violations fail CI, while existing ones are tracked and their
     eventual fix is reported as good news.

Usage:
    python3 check_include_layering.py [OPTIONS] BUILD_DIR

    BUILD_DIR must contain a compile_commands.json (e.g. a Zephyr/CMake
    build directory such as build/hello_world).

Examples:
    # Human-readable report, compared against the committed baseline
    python3 check_include_layering.py build/hello_world

    # Regenerate the baseline after intentionally accepting new violations
    python3 check_include_layering.py --update-baseline build/hello_world

    # Ignore the baseline entirely and fail on every violation
    python3 check_include_layering.py --no-baseline build/hello_world

    # Machine-readable report for tooling
    python3 check_include_layering.py --json build/hello_world > report.json

Exit status:
    0  no (new) violations
    1  new violations found (or, with --no-baseline, any violations)
    2  usage / environment error (bad build dir, rules file, etc.)
'''

import argparse
import json
import os
import re
import shlex
import subprocess
import sys
from collections import defaultdict, deque
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

try:
    import yaml
except ImportError:
    sys.exit(
        "error: PyYAML is required (pip install pyyaml) to parse the rules file"
    )


SCRIPT_DIR = Path(__file__).resolve().parent
DEFAULT_RULES = SCRIPT_DIR / "include_layers.yml"
DEFAULT_BASELINE = SCRIPT_DIR / "include_layers_baseline.json"


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------


def parse_args(argv=None):
    p = argparse.ArgumentParser(
        prog="check_include_layering.py",
        description=(
            "Check that scoped source files do not transitively #include "
            "headers from forbidden layers (e.g. kernel -> vendor HAL)."
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument(
        "build_dir",
        help="Build directory containing compile_commands.json",
    )
    p.add_argument(
        "--rules",
        default=str(DEFAULT_RULES),
        metavar="FILE",
        help=f"YAML rules file (default: {DEFAULT_RULES.name} next to this script)",
    )
    p.add_argument(
        "--zephyr-root",
        metavar="DIR",
        help=(
            "Path to the zephyr repository root. Scope patterns in the rules "
            "file are matched relative to this directory. Default: auto-"
            "detected from this script's own location (zephyr/scripts/...)."
        ),
    )
    p.add_argument(
        "--workspace-root",
        metavar="DIR",
        help=(
            "Path to the west workspace root. Forbidden patterns in the "
            "rules file are matched relative to this directory. Default: "
            "'west topdir', falling back to the build directory's parent."
        ),
    )

    baseline = p.add_mutually_exclusive_group()
    baseline.add_argument(
        "--update-baseline",
        action="store_true",
        help=(
            "Write the current violation set to --baseline instead of "
            "comparing against it. Always exits 0 on success."
        ),
    )
    baseline.add_argument(
        "--no-baseline",
        action="store_true",
        help=(
            "Ignore the baseline file entirely: report every violation and "
            "exit non-zero if any are found."
        ),
    )
    p.add_argument(
        "--baseline",
        default=str(DEFAULT_BASELINE),
        metavar="FILE",
        help=f"Baseline file (default: {DEFAULT_BASELINE.name} next to this script)",
    )

    p.add_argument(
        "--json",
        action="store_true",
        help="Emit a machine-readable JSON report on stdout instead of text",
    )
    p.add_argument(
        "-j",
        "--jobs",
        type=int,
        default=os.cpu_count() or 4,
        metavar="N",
        help="Parallel -H compile workers (default: number of CPUs)",
    )
    p.add_argument(
        "-q",
        "--quiet",
        action="store_true",
        help="Suppress progress output on stderr",
    )
    return p.parse_args(argv)


# ---------------------------------------------------------------------------
# Root discovery
# ---------------------------------------------------------------------------


def find_zephyr_root(explicit):
    if explicit:
        root = Path(explicit).resolve()
    else:
        # This script lives in <zephyr>/scripts/check_include_layering.py
        root = SCRIPT_DIR.parent
    if not (root / "kernel" / "include").exists() and not (root / "VERSION").exists():
        print(
            f"warning: {root} does not look like a zephyr repo root "
            "(no VERSION file) -- continuing anyway",
            file=sys.stderr,
        )
    return root


def find_workspace_root(explicit, build_dir, zephyr_root):
    if explicit:
        return Path(explicit).resolve()
    r = subprocess.run(
        ["west", "topdir"],
        cwd=str(build_dir),
        capture_output=True,
        text=True,
    )
    if r.returncode == 0 and r.stdout.strip():
        return Path(r.stdout.strip()).resolve()
    # Fall back: assume the build directory lives directly under the
    # workspace root (e.g. <workspace>/build-foo), like zephyr_root does.
    return build_dir.resolve().parent


# ---------------------------------------------------------------------------
# Rules
# ---------------------------------------------------------------------------


class Rule:
    __slots__ = ("name", "description", "scope", "forbidden")

    def __init__(self, name, description, scope, forbidden):
        self.name = name
        self.description = description
        self.scope = scope
        self.forbidden = forbidden


def load_rules(path):
    path = Path(path)
    if not path.exists():
        sys.exit(f"error: rules file not found: {path}")
    with open(path) as f:
        data = yaml.safe_load(f) or []
    if not isinstance(data, list):
        sys.exit(f"error: {path}: top level must be a list of rules")

    rules = []
    for i, item in enumerate(data):
        if "scope" not in item or "forbidden" not in item:
            sys.exit(f"error: {path}: rule #{i} is missing 'scope' or 'forbidden'")
        scope = item["scope"]
        forbidden = item["forbidden"]
        scope = [scope] if isinstance(scope, str) else list(scope)
        forbidden = [forbidden] if isinstance(forbidden, str) else list(forbidden)
        rules.append(
            Rule(
                name=item.get("name", f"rule-{i}"),
                description=item.get("description", ""),
                scope=scope,
                forbidden=forbidden,
            )
        )
    if not rules:
        sys.exit(f"error: {path}: no rules defined")
    return rules


# ---------------------------------------------------------------------------
# Glob-ish pattern matching
#
# Patterns use a small glob dialect: "**" matches any number of path
# segments (including zero), "*" matches within a single segment, "?"
# matches a single character. Everything else is literal.
#
# A pattern is tried, in order, against:
#   1. the candidate path relative to its "natural" root (full match) --
#      this is the documented, precise form, e.g. "kernel/**" relative to
#      the zephyr root, or "modules/hal/**" relative to the workspace root.
#   2. as a path-boundary-anchored *fragment* of the absolute path, so
#      that rules also work when a rooted match isn't available or when a
#      pattern is intentionally written as a bare fragment (e.g. a vendor
#      directory name that could appear at different depths).
# ---------------------------------------------------------------------------


def _glob_body_to_regex(pattern):
    parts = []
    i, n = 0, len(pattern)
    while i < n:
        c = pattern[i]
        if pattern[i : i + 2] == "**":
            parts.append(".*")
            i += 2
        elif c == "*":
            parts.append("[^/]*")
            i += 1
        elif c == "?":
            parts.append("[^/]")
            i += 1
        else:
            parts.append(re.escape(c))
            i += 1
    return "".join(parts)


def compile_pattern(pattern):
    body = _glob_body_to_regex(pattern.strip("/"))
    full = re.compile(r"^(?:" + body + r")$")
    fragment = re.compile(r"(?:^|/)(?:" + body + r")$")
    return full, fragment


def path_matches(abs_path, root, pattern_cache, pattern):
    """Return True if `abs_path` matches `pattern`.

    `root` (a Path or None) is the preferred relative-to base -- e.g. the
    zephyr root for scope patterns, the workspace root for forbidden
    patterns. When `abs_path` lives under `root`, matching is a *precise*
    full match of `pattern` against the root-relative path (this is what
    keeps e.g. "zephyr/drivers/**" from spuriously matching public API
    headers under zephyr/include/zephyr/drivers/, which merely happen to
    contain that path fragment).

    Only when `abs_path` cannot be expressed relative to `root` at all (it
    lives outside it -- e.g. a different repo layout, or `root` wasn't
    supplied) do we fall back to fragment matching against the absolute
    path, anchored at a '/' boundary, to support patterns intentionally
    written as bare fragments.
    """
    full_re, frag_re = pattern_cache[pattern]
    rel = None
    if root is not None:
        try:
            rel = Path(abs_path).relative_to(root).as_posix()
        except ValueError:
            rel = None
    if rel is not None:
        return bool(full_re.fullmatch(rel))
    return bool(frag_re.search(abs_path.replace(os.sep, "/")))


# ---------------------------------------------------------------------------
# compile_commands.json handling + -H parsing
# (approach follows extra/dep-analyzer/include_graph.py's -H/GCC parsing,
# reimplemented here to keep this script self-contained / import-free)
# ---------------------------------------------------------------------------


def make_H_command(entry):
    """Strip -c and -o <file> from the original compile command and append
    -H -fsyntax-only. Returns (argv_list, cwd)."""
    parts = shlex.split(entry["command"])
    out = []
    skip = False
    for part in parts:
        if skip:
            skip = False
            continue
        if part == "-o":
            skip = True
            continue
        if part == "-c":
            continue
        out.append(part)
    out += ["-H", "-fsyntax-only"]
    return out, entry["directory"]


def run_one(entry):
    cmd, cwd = make_H_command(entry)
    r = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True)
    return entry["file"], r.stderr, r.returncode


_H_LINE_RE = re.compile(r"^(\.+) (.+)$")


def parse_H_stderr(source, stderr):
    """Parse GCC -H output into a set of (parent, child) direct-#include
    edges. Depth is the number of leading dots (1 = included directly by
    `source`). Paths are normalized (no './'/'../' noise) but not
    symlink-resolved."""
    edges = set()
    stack = [os.path.normpath(source)]
    for line in stderr.splitlines():
        m = _H_LINE_RE.match(line)
        if not m:
            continue
        depth = len(m.group(1))
        child = os.path.normpath(m.group(2).strip())
        stack = stack[:depth]
        if stack:
            edges.add((stack[-1], child))
        stack.append(child)
    return edges


def build_graph(compile_commands, workers, quiet):
    """Run -H for each entry in `compile_commands` (a list of
    compile_commands.json entries) and return (edges, failures) where edges
    is a set of (parent, child) absolute-path tuples and failures is a list
    of (source, stderr) for TUs that failed to compile even with
    -fsyntax-only."""
    edges = set()
    failures = []
    total = len(compile_commands)
    if total == 0:
        return edges, failures

    if not quiet:
        print(
            f"Running {total} compilation(s) with -H -fsyntax-only "
            f"using {workers} worker(s) ...",
            file=sys.stderr,
        )

    with ThreadPoolExecutor(max_workers=workers) as ex:
        futures = {ex.submit(run_one, entry): entry for entry in compile_commands}
        done = 0
        for fut in as_completed(futures):
            done += 1
            if not quiet:
                print(f"\r  {done}/{total}", end="", file=sys.stderr, flush=True)
            source, stderr, rc = fut.result()
            if rc != 0:
                failures.append((source, stderr))
            for src, dst in parse_H_stderr(source, stderr):
                edges.add((src, dst))

    if not quiet:
        print(f"\r  {total}/{total} done", file=sys.stderr)
    return edges, failures


# ---------------------------------------------------------------------------
# Reachability + shortest chain
# ---------------------------------------------------------------------------


def shortest_chains_from(root, adjacency):
    """BFS from `root` over `adjacency` (dict: node -> set(children)).
    Returns dict: reachable_node -> chain (list of nodes from root to that
    node, inclusive), using the shortest (fewest hops) chain found."""
    chains = {root: [root]}
    q = deque([root])
    while q:
        node = q.popleft()
        for child in adjacency.get(node, ()):
            if child not in chains:
                chains[child] = chains[node] + [child]
                q.append(child)
    return chains


# ---------------------------------------------------------------------------
# Violation detection
# ---------------------------------------------------------------------------


def relpath_for_report(abs_path, workspace_root):
    try:
        return Path(abs_path).relative_to(workspace_root).as_posix()
    except ValueError:
        return abs_path


def find_violations(rules, compiled_tus, edges, zephyr_root, workspace_root):
    """Returns a list of violation dicts:
        {rule, tu, header, chain}
    `tu` and `header` and `chain` entries are workspace-root-relative paths
    where possible. One violation is reported per (rule, forbidden header)
    pair -- the example TU/chain chosen is the shortest one found across all
    scope-matching TUs, since a header being reachable at all from the scope
    is the fact we're flagging, and a single concrete chain is enough to act
    on it. (Compiling one violation per (rule, TU, header) would multiply
    into thousands of near-duplicate entries for a broadly shared header
    such as a HAL master include.)
    """
    adjacency = defaultdict(set)
    for src, dst in edges:
        adjacency[src].add(dst)

    pattern_cache = {}
    for rule in rules:
        for pat in rule.scope + rule.forbidden:
            if pat not in pattern_cache:
                pattern_cache[pat] = compile_pattern(pat)

    # best[(rule.name, header)] = (chain_length, tu, chain)
    best = {}

    for rule in rules:
        scope_tus = [
            tu
            for tu in compiled_tus
            if any(
                path_matches(tu, zephyr_root, pattern_cache, pat)
                for pat in rule.scope
            )
        ]
        for tu in scope_tus:
            chains = shortest_chains_from(tu, adjacency)
            for node, chain in chains.items():
                if node == tu:
                    continue
                if any(
                    path_matches(node, workspace_root, pattern_cache, pat)
                    for pat in rule.forbidden
                ):
                    key = (rule.name, node)
                    length = len(chain)
                    if key not in best or length < best[key][0]:
                        best[key] = (length, tu, chain)

    violations = []
    for (rule_name, header), (_length, tu, chain) in best.items():
        violations.append(
            {
                "rule": rule_name,
                "tu": relpath_for_report(tu, workspace_root),
                "header": relpath_for_report(header, workspace_root),
                "chain": [relpath_for_report(p, workspace_root) for p in chain],
            }
        )
    violations.sort(key=lambda v: (v["rule"], v["header"]))
    return violations


# ---------------------------------------------------------------------------
# Baseline
# ---------------------------------------------------------------------------


def violation_key(v):
    return (v["rule"], v["header"])


def load_baseline(path):
    path = Path(path)
    if not path.exists():
        return None
    with open(path) as f:
        data = json.load(f)
    return {(v["rule"], v["header"]): v for v in data.get("violations", [])}


def write_baseline(path, violations):
    payload = {
        "$schema-comment": (
            "Baseline of accepted include-layering violations. Generated by "
            "check_include_layering.py --update-baseline. New violations "
            "not listed here will fail the check; violations listed here "
            "that no longer reproduce are reported as fixed (informational)."
        ),
        "violations": violations,
    }
    Path(path).write_text(json.dumps(payload, indent=2, sort_keys=False) + "\n")


# ---------------------------------------------------------------------------
# Reporting
# ---------------------------------------------------------------------------


def format_chain(chain):
    return " -> ".join(chain)


def print_human_report(rules, current, new, fixed, unchanged, baseline_mode, out):
    by_rule = defaultdict(list)
    for v in current:
        by_rule[v["rule"]].append(v)

    print("Include layering check", file=out)
    print("=======================", file=out)
    for rule in rules:
        vs = by_rule.get(rule.name, [])
        print(f"\nRule: {rule.name}  ({len(vs)} violation(s))", file=out)
        if rule.description:
            print(f"  {rule.description.strip()}", file=out)
        print(f"  scope:     {rule.scope}", file=out)
        print(f"  forbidden: {rule.forbidden}", file=out)

    new_set = {violation_key(v) for v in new}
    if new:
        print(f"\n{'='*70}", file=out)
        print(f"NEW VIOLATIONS ({len(new)})", file=out)
        print("=" * 70, file=out)
        for v in current:
            if violation_key(v) not in new_set:
                continue
            print(f"\n[{v['rule']}] {v['tu']}", file=out)
            print(f"  reaches forbidden header: {v['header']}", file=out)
            print(f"  example chain:", file=out)
            print(f"    {format_chain(v['chain'])}", file=out)

    if baseline_mode:
        if fixed:
            print(f"\n{'-'*70}", file=out)
            print(f"FIXED since baseline ({len(fixed)}) -- informational:", file=out)
            for v in fixed:
                print(f"  [{v['rule']}] {v['header']}  (was via {v['tu']})", file=out)
        if unchanged:
            print(
                f"\n{len(unchanged)} pre-existing (baselined) violation(s) unchanged.",
                file=out,
            )

    print(f"\n{'='*70}", file=out)
    total = len(current)
    if baseline_mode:
        print(
            f"Summary: {total} total violation(s), {len(new)} new, "
            f"{len(fixed)} fixed, {len(unchanged)} baselined.",
            file=out,
        )
    else:
        print(f"Summary: {total} total violation(s) (no baseline).", file=out)


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main(argv=None):
    args = parse_args(argv)

    build_dir = Path(args.build_dir).resolve()
    cc_path = build_dir / "compile_commands.json"
    if not cc_path.exists():
        sys.exit(f"error: {cc_path} not found")

    zephyr_root = find_zephyr_root(args.zephyr_root)
    workspace_root = find_workspace_root(args.workspace_root, build_dir, zephyr_root)

    rules = load_rules(args.rules)

    compile_commands = json.loads(cc_path.read_text())

    # Optimization: only TUs that fall in at least one rule's scope need to
    # be compiled with -H; their transitive -H output already contains the
    # full reachable subgraph for that TU, so no other TU's edges are
    # required to evaluate these rules.
    pattern_cache = {}
    for rule in rules:
        for pat in rule.scope:
            pattern_cache.setdefault(pat, compile_pattern(pat))

    scoped_entries = [
        e
        for e in compile_commands
        if any(
            path_matches(
                os.path.normpath(e["file"]), zephyr_root, pattern_cache, pat
            )
            for rule in rules
            for pat in rule.scope
        )
    ]

    if not scoped_entries:
        print(
            "warning: no translation units in compile_commands.json matched "
            "any rule scope -- nothing to check",
            file=sys.stderr,
        )

    edges, failures = build_graph(scoped_entries, args.jobs, args.quiet)
    for source, stderr in failures:
        print(
            f"warning: -fsyntax-only failed for {source}, its include tree "
            f"may be incomplete:\n{stderr.strip()[-500:]}",
            file=sys.stderr,
        )

    compiled_tus = [os.path.normpath(e["file"]) for e in scoped_entries]
    current = find_violations(rules, compiled_tus, edges, zephyr_root, workspace_root)

    if args.update_baseline:
        write_baseline(args.baseline, current)
        if not args.quiet:
            print(
                f"Wrote {len(current)} violation(s) to baseline: {args.baseline}",
                file=sys.stderr,
            )
        return 0

    if args.no_baseline:
        new, fixed, unchanged = current, [], []
        baseline_mode = False
    else:
        baseline = load_baseline(args.baseline)
        if baseline is None:
            print(
                f"warning: no baseline file at {args.baseline}; treating all "
                "violations as new. Run with --update-baseline to create one.",
                file=sys.stderr,
            )
            baseline = {}
        current_keys = {violation_key(v) for v in current}
        new = [v for v in current if violation_key(v) not in baseline]
        fixed = [v for k, v in baseline.items() if k not in current_keys]
        unchanged = [v for v in current if violation_key(v) in baseline]
        baseline_mode = True

    if args.json:
        report = {
            "build_dir": str(build_dir),
            "zephyr_root": str(zephyr_root),
            "workspace_root": str(workspace_root),
            "rules": [
                {
                    "name": r.name,
                    "description": r.description,
                    "scope": r.scope,
                    "forbidden": r.forbidden,
                }
                for r in rules
            ],
            "baseline_mode": baseline_mode,
            "violations": current,
            "new": new,
            "fixed": fixed,
            "unchanged": unchanged if baseline_mode else [],
        }
        print(json.dumps(report, indent=2))
    else:
        print_human_report(
            rules, current, new, fixed, unchanged, baseline_mode, sys.stdout
        )

    return 1 if new else 0


if __name__ == "__main__":
    sys.exit(main())
