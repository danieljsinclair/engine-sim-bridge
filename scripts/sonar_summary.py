#!/usr/bin/env python3
"""Display a SonarCloud issue summary from a cached or live API report.

The report is the JSON returned by ``/api/issues/search`` (filtered to
``statuses=OPEN`` and requested with ``facets=impactSeverities`` by the
Makefile curl). Two severity taxonomies are available:

* **Severity (default)**: BLOCKER / HIGH / MEDIUM / LOW / INFO. This is the
  new SonarCloud "impacts" taxonomy. The breakdown is read STRAIGHT from the
  API's ``impactSeverities`` facet -- the SAME server-side counting the
  SonarCloud dashboard uses -- rather than re-derived per issue in Python, so
  local buckets always equal the dashboard's severity widget. To match the
  dashboard widget exactly (which counts OPEN ∪ REMOVED issues), the optional
  ``--removed-facet <path>`` supplies a second ``impactSeverities`` facet for
  the ``resolutions=REMOVED`` set; the two facets are summed.
* **Type severity** (``--type-severity``): the older CRITICAL / MAJOR / MINOR /
  INFO ``severity`` field. Off by default because it does NOT sum to the
  headline total on the modern dashboard; pass the flag to show it alongside.

Top issues are always sorted "critical-first" (highest impact severity first,
ties broken by type severity, then by rule) so the most important findings lead.

Usage:
    sonar_summary.py <report.json> [--type-severity] [--removed-facet <path>]
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
GREY = "\033[90m"
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


def impact_severity_facet(data: dict) -> dict | None:
    """Return the ``impactSeverities`` facet as a {severity: count} dict.

    The SonarCloud dashboard's severity widget is computed server-side from
    this same facet, so reading it directly (instead of re-deriving it per
    issue in Python) keeps local buckets identical to the dashboard. Returns
    ``None`` when the facet is absent (e.g. an old cached report with no
    ``facets=`` in the curl), so callers can fall back to per-issue counting.
    """
    for facet in data.get("facets") or []:
        if facet.get("property") == "impactSeverities":
            return {v.get("val"): v.get("count", 0) for v in facet.get("values") or []}
    return None


def merge_facets(*facets: dict | None) -> dict:
    """Sum multiple ``{severity: count}`` facet dicts (OPEN ∪ REMOVED, etc.).

    ``None`` facets are skipped. Returns a dict with all ``IMPACT_ORDER`` keys.
    """
    merged: dict = {sev: 0 for sev in IMPACT_ORDER}
    for facet in facets:
        if not facet:
            continue
        for sev in IMPACT_ORDER:
            merged[sev] += facet.get(sev, 0)
    return merged


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
    """Print a single top issue, coloured by its highest impact severity.

    Format: ``[SEVERITY] [rule] file:line - message`` -- the rule (e.g.
    ``cpp:S5421``) lets the reader jump straight to the SonarCloud rule page.
    """
    sev = highest_impact_severity(issue) or type_severity(issue)
    colour = severity_colour(sev if sev in IMPACT_ORDER else "")
    message = (issue.get("message") or "")[:70]
    component = (issue.get("component") or "").split(":")[-1]
    line = issue.get("line")
    location = component + (f":{line}" if line else "")
    rule = issue.get("rule") or "?"
    print(f"    {colour}[{sev:<6}] [{rule}]{RESET} {location} - {message}")


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


def display_summary(
    data: dict,
    show_type_severity: bool = False,
    label: str = "SonarCloud",
    removed_data: dict | None = None,
) -> None:
    """Render the issue summary. ``label`` names the repo for the banner.

    The severity breakdown is read from the API's ``impactSeverities`` facet
    (the dashboard's own server-side counting). ``removed_data`` carries a
    second report whose ``impactSeverities`` facet covers the ``resolutions=
    REMOVED`` set; summing the two makes the breakdown match the dashboard
    severity widget, which counts OPEN ∪ REMOVED issues. When no facet is
    present (old cached report), it falls back to highest-impact-per-issue.
    """
    issues = data.get("issues") or []
    fetched = len(issues)

    open_facet = impact_severity_facet(data)
    removed_facet = impact_severity_facet(removed_data) if removed_data else None

    if open_facet is not None:
        impact_counts = merge_facets(open_facet, removed_facet)
        facet_source = "impactSeverities facet"
    else:
        # Legacy cached report with no facet: fall back to per-issue rollup of
        # the OPEN set only (REMOVED issues are not in this report).
        impact_counts = count_by_impact(issues)
        facet_source = "highest impact per issue (no facet in report)"

    total = sum(impact_counts.values())

    print("")
    print(f"=== {label} SonarCloud Issues ===")

    if total == 0:
        print(f"  {GREEN}No open issues found.{RESET}")
        print("")
        return

    open_total = sum(open_facet.values()) if open_facet else fetched
    removed_total = sum(removed_facet.values()) if removed_facet else 0

    # HEADLINE: lead with Open issues in red (prominent), then the total +
    # relationship to removed. Mirrors how the coverage section leads with its
    # overall number. REMOVED = issues whose source was deleted (dead code),
    # distinct from FIXED/FALSE-POSITIVE/WONTFIX.
    if removed_total:
        print(f"  {RED}{BOLD}Open issues: {open_total}{RESET}"
              f"  {GREY}(Total {total} - {removed_total} removed){RESET}")
    elif fetched and fetched < data.get("total", 0):
        print(f"  {RED}{BOLD}Open issues: {open_total}{RESET}"
              f"  {ORANGE}(showing {fetched} of {data.get('total')}){RESET}")
    else:
        print(f"  {RED}{BOLD}Open issues: {open_total}{RESET}"
              f"  {GREY}(Total {total}){RESET}")

    print("")
    print(f"  Issues by severity ({facet_source}):")
    format_severity_line("Severity", impact_counts, IMPACT_ORDER)

    if show_type_severity:
        type_counts = count_by_type(issues)
        print("")
        print("  Issues by type severity (legacy field, OPEN set only):")
        format_severity_line("Type severity", type_counts, TYPE_ORDER)

    print("")
    print("  Top issues (critical-first, OPEN set):")
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
        print("Usage: sonar_summary.py <report.json> [--type-severity] [--label NAME] [--removed-facet <path>]", file=sys.stderr)
        sys.exit(2)
    report_path = argv[1]
    show_type_severity = "--type-severity" in argv[2:]
    label = "SonarCloud"
    removed_facet_path = None
    rest = argv[2:]
    i = 0
    while i < len(rest):
        if rest[i] == "--label" and i + 1 < len(rest):
            label = rest[i + 1]
            i += 2
        elif rest[i] == "--removed-facet" and i + 1 < len(rest):
            removed_facet_path = rest[i + 1]
            i += 2
        else:
            i += 1
    return report_path, show_type_severity, label, removed_facet_path


def main(argv: list | None = None) -> int:
    # Avoid BrokenPipeError when the caller pipes output through `head`.
    try:
        os.set_blocking(sys.stdout.fileno(), True)
    except (OSError, AttributeError, ValueError):
        pass

    report_path, show_type_severity, label, removed_facet_path = _parse_args(
        sys.argv if argv is None else argv
    )

    data = load_report(report_path)
    if data is None:
        print("  No report yet. Run: make sonar-scan")
        return 0

    removed_data = load_report(removed_facet_path) if removed_facet_path else None

    display_summary(
        data,
        show_type_severity=show_type_severity,
        label=label,
        removed_data=removed_data,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
