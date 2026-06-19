#!/usr/bin/env python3
"""Display a SonarCloud issue summary from a cached or live API report.

The report is the JSON returned by ``/api/issues/search`` (filtered to
``statuses=OPEN`` by the Makefile curl). Two severity taxonomies are available:

* **Severity (default)**: BLOCKER / HIGH / MEDIUM / LOW / INFO. This is the
  new SonarCloud "impacts" taxonomy. Each issue is counted ONCE under its
  HIGHEST-impact severity, so the breakdown sums exactly to the headline total
  (matches the dashboard's severity breakdown).
* **Type severity** (``--type-severity``): the older CRITICAL / MAJOR / MINOR /
  INFO ``severity`` field. Off by default because it does NOT sum to the
  headline total on the modern dashboard; pass the flag to show it alongside.

Top issues are always sorted "critical-first" (highest impact severity first,
ties broken by type severity, then by rule) so the most important findings lead.

Usage:
    sonar_summary.py <report.json> [--type-severity]
"""
from __future__ import annotations

import json
import os
import sys
from typing import Iterable

# ANSI colours.
RED = "\033[31m"
ORANGE = "\033[38;5;208m"
YELLOW = "\033[33m"
GREEN = "\033[32m"
CYAN = "\033[36m"
BOLD = "\033[1m"
RESET = "\033[0m"

# Display order, worst-first. Both taxonomies use the same colour ramp.
IMPACT_ORDER = ["BLOCKER", "HIGH", "MEDIUM", "LOW", "INFO"]
TYPE_ORDER = ["BLOCKER", "CRITICAL", "MAJOR", "MINOR", "INFO"]

# Rank by severity (higher = worse). Two separate scales so impact and type
# severities are comparable within their own taxonomy.
IMPACT_RANK = {sev: len(IMPACT_ORDER) - i for i, sev in enumerate(IMPACT_ORDER)}
TYPE_RANK = {sev: len(TYPE_ORDER) - i for i, sev in enumerate(TYPE_ORDER)}


def highest_impact_severity(issue: dict) -> str | None:
    """Return the highest-impact severity for an issue, or ``None`` if no impacts.

    SonarCloud may attach multiple impacts (e.g. MAINTAINABILITY + RELIABILITY)
    to one issue; the dashboard rolls them up by the most severe. We mirror
    that so our breakdown sums to the headline count.
    """
    impacts = issue.get("impacts") or []
    best = None
    best_rank = -1
    for imp in impacts:
        sev = imp.get("severity") if isinstance(imp, dict) else None
        rank = IMPACT_RANK.get(sev, -1)
        if sev is not None and rank > best_rank:
            best = sev
            best_rank = rank
    return best


def type_severity(issue: dict) -> str:
    """Return the legacy ``severity`` field, defaulting to UNKNOWN if absent."""
    return issue.get("severity") or "UNKNOWN"


def count_by_impact(issues: Iterable[dict]) -> dict:
    """Count issues once each by their highest-impact severity."""
    counts: dict = {sev: 0 for sev in IMPACT_ORDER}
    for issue in issues:
        sev = highest_impact_severity(issue)
        if sev in counts:
            counts[sev] += 1
    return counts


def count_by_type(issues: Iterable[dict]) -> dict:
    """Count issues by the legacy ``severity`` field."""
    counts: dict = {sev: 0 for sev in TYPE_ORDER}
    for issue in issues:
        counts[type_severity(issue)] = counts.get(type_severity(issue), 0) + 1
    return counts


def severity_colour(severity: str) -> str:
    """Map a severity (either taxonomy) to an ANSI colour."""
    if severity in ("BLOCKER", "CRITICAL"):
        return RED
    if severity in ("HIGH", "MAJOR"):
        return ORANGE
    if severity in ("MEDIUM", "MINOR"):
        return YELLOW
    if severity == "LOW":
        return GREEN
    if severity == "INFO":
        return CYAN
    return GREEN


def headline_colour(counts: dict) -> str:
    """Colour the headline by the worst present severity."""
    for sev in IMPACT_ORDER:
        if counts.get(sev, 0) > 0:
            return severity_colour(sev)
    return GREEN


def format_severity_line(label: str, counts: dict, order: list) -> None:
    """Print one severity-count line per entry in ``order``."""
    for sev in order:
        colour = severity_colour(sev)
        print(f"    {colour}● {label:<14}: {sev:<8} {counts.get(sev, 0)}{RESET}")


def format_issue_line(issue: dict) -> None:
    """Print a single top issue, coloured by its highest impact severity."""
    sev = highest_impact_severity(issue) or type_severity(issue)
    colour = severity_colour(sev if sev in IMPACT_ORDER else "")
    message = (issue.get("message") or "")[:70]
    component = (issue.get("component") or "").split(":")[-1]
    line = issue.get("line")
    location = component + (f":{line}" if line else "")
    print(f"    {colour}[{sev:<6}]{RESET} {location} - {message}")


def sort_critical_first(issues: Iterable[dict]) -> list:
    """Sort issues worst-first by impact severity, then type severity, then rule."""
    return sorted(
        issues,
        key=lambda i: (
            IMPACT_RANK.get(highest_impact_severity(i), -1),
            TYPE_RANK.get(type_severity(i), -1),
            i.get("rule") or "",
        ),
        reverse=True,
    )


def display_summary(data: dict, show_type_severity: bool = False, label: str = "SonarCloud") -> None:
    """Render the issue summary. ``label`` names the repo for the banner."""
    issues = data.get("issues") or []
    total = data.get("total", len(issues))
    fetched = len(issues)
    impact_counts = count_by_impact(issues)

    print("")
    print(f"=== {label} SonarCloud Issues ===")

    if total == 0:
        print(f"  {GREEN}No open issues found.{RESET}")
        print("")
        return

    print(f"  {headline_colour(impact_counts)}Total open issues: {total}{RESET}", end="")
    if fetched < total:
        print(f"  {ORANGE}(showing {fetched} of {total}){RESET}")
    else:
        print("")

    print("")
    print("  Issues by severity (highest impact per issue):")
    format_severity_line("Severity", impact_counts, IMPACT_ORDER)

    if show_type_severity:
        type_counts = count_by_type(issues)
        print("")
        print("  Issues by type severity (legacy field):")
        format_severity_line("Type severity", type_counts, TYPE_ORDER)

    print("")
    print("  Top issues (critical-first):")
    for issue in sort_critical_first(issues)[:10]:
        format_issue_line(issue)

    print("")


def load_report(path: str) -> dict | None:
    """Load and validate the SonarCloud report JSON. Returns ``None`` on error."""
    try:
        with open(path, "r", encoding="utf-8") as handle:
            return json.load(handle)
    except (FileNotFoundError, json.JSONDecodeError):
        return None


def _parse_args(argv: list) -> tuple:
    if len(argv) < 2:
        print("Usage: sonar_summary.py <report.json> [--type-severity] [--label NAME]", file=sys.stderr)
        sys.exit(2)
    report_path = argv[1]
    show_type_severity = "--type-severity" in argv[2:]
    label = "SonarCloud"
    rest = argv[2:]
    i = 0
    while i < len(rest):
        if rest[i] == "--label" and i + 1 < len(rest):
            label = rest[i + 1]
            i += 2
        else:
            i += 1
    return report_path, show_type_severity, label


def main(argv: list | None = None) -> int:
    # Avoid BrokenPipeError when the caller pipes output through `head`.
    try:
        os.set_blocking(sys.stdout.fileno(), True)
    except (OSError, AttributeError, ValueError):
        pass

    report_path, show_type_severity, label = _parse_args(sys.argv if argv is None else argv)

    data = load_report(report_path)
    if data is None:
        print("  No report yet. Run: make sonar-scan")
        return 0

    display_summary(data, show_type_severity=show_type_severity, label=label)
    return 0


if __name__ == "__main__":
    sys.exit(main())
