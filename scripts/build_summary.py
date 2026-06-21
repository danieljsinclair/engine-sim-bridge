#!/usr/bin/env python3
"""Emit a single compact, coloured HEADLINE line for one repo at end-of-make.

This is the END-OF-MAKE summary (the "russian doll" headline). Each repo's
``summary`` Makefile target calls this script ONCE for ITS OWN reports, then
(for cli/app) recurses into its submodule via ``$(MAKE) -C <sub> summary`` so
the nesting prints top-down:

    app line
      cli line
        bridge line

The line is one scannable row:

    [repo] tests: <passed>/<total> | cov: <X%> <covered>/<total> | sonar: open <O> / total <T>

Fields for which no data exists are OMITTED gracefully (never fabricated, never
crashed). Colours are EMIT DELIBERATELY here (plain numbers are extracted from
the report files and re-emitted with ANSI) -- we do NOT grep-and-preserve
source ANSI because grep mangles colours. Controlling emission gives one
consistent palette and dodges that entirely.

Colour rules (matching coverage_block.py / sonar_summary.py):

    tests       : GREEN if all passed, RED if any failed.
    coverage %  : GREEN >=80 / CYAN >=60 / YELLOW >=40 / ORANGE >0 / RED =0.
    sonar       : RED if any BLOCKER severity present, else YELLOW by open count.

DATA SOURCES -- plain numbers grepped from existing report files:

    Tests pass/fail
        ctest writes ``<build>/Testing/Temporary/LastTest.log`` (per-test
        "Test Passed."/"Test Failed." markers) and, when teed, an
        ``N% tests passed, M tests failed out of N`` summary line. This script
        reads whichever greppable log the Makefile points it at, strips ANSI,
        and aggregates ALL summary lines (a repo may run ctest in tiers). If
        no summary line is present it falls back to counting the per-test
        Passed/Failed markers.

    Coverage
        Prefer the cached SonarCloud measures JSON (``sonar-measures.json`` --
        the SAME number coverage_block.py shows as the live SonarCloud headline,
        cached so this is fast and never re-queries). Fall back to the local
        lcov.info (LF/LH aggregated over /src/) or xccov JSON.

    Sonar open/total
        The cached ``sonar-report.json`` (the ``/api/issues/search?statuses=
        OPEN`` response the sonar-summary target already curls). ``open`` is
        derived from the ``impactSeverities`` facet when present (the
        dashboard's own server-side count), else from the per-issue impacts/
        legacy severity. ``total`` is ``open + removed`` -- the SAME OPEN union
        REMOVED definition the SonarCloud dashboard and the detailed
        ``sonar_summary.py`` use (the dashboard severity widget counts OPEN union
        REMOVED issues). The REMOVED count comes from a second cached report
        (``--removed-facet <path>``, the ``resolutions=REMOVED&facets=
        impactSeverities`` response the sonar-summary target curls); when that
        file is absent/empty ``total`` falls back to ``open`` (and a GREY
        ``(open-only)`` marker notes the fallback so a redundant-looking
        ``open X / total X`` is never mistaken for the merged total).

Usage:

    build_summary.py --label "[engine-sim-bridge]" \\
        [--test-log PATH] \\
        [--cov-measures PATH] [--local-cov PATH --local-type lcov|xccov] \\
        [--sonar-report PATH] [--sonar-measures PATH] \\
        [--removed-facet PATH]

Exit codes: 0 always (a missing file / parse failure is reported in-line,
never a crash -- this is a display helper, not a build step).
"""
import argparse
import json
import os
import re
import sys

# ---------------------------------------------------------------------------
# ANSI palette (deliberate emission -- same codes as coverage_block.py /
# sonar_summary.py so the whole report reads as one measurement view).
# ---------------------------------------------------------------------------
RED = '\033[31m'
ORANGE = '\033[38;5;208m'
YELLOW = '\033[33m'
GREEN = '\033[32m'
CYAN = '\033[36m'
GREY = '\033[90m'
BOLD = '\033[1m'
RESET = '\033[0m'

_ANSI_RE = re.compile(r'\x1b\[[0-9;]*m')

# Matches "100% tests passed, 0 tests failed out of 35" (with or without ANSI).
_SUMMARY_RE = re.compile(
    r'(\d+)%\s*tests?\s*passed,?\s*(\d+)\s*tests?\s*failed,?\s*out\s*of\s*(\d+)',
    re.IGNORECASE,
)


def _strip_ansi(text):
    """Remove ANSI escape sequences from ``text``."""
    return _ANSI_RE.sub('', text)


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------
def parse_tests(log_path):
    """Return (passed, total) aggregated from a ctest log, or None if unavailable.

    Aggregates ALL ``N% tests passed, M tests failed out of N`` summary lines
    (a repo may run ctest in tiers -- each tier's run is appended to the log).
    When no summary line is present, falls back to counting the per-test
    ``Test Passed.`` / ``Test Failed.`` markers ctest writes to LastTest.log.
    Returns None when the log is absent or carries no test markers at all so
    the caller OMITS the tests field rather than fabricating a number.
    """
    if not log_path or not os.path.isfile(log_path):
        return None
    try:
        with open(log_path, errors='replace') as handle:
            raw = handle.read()
    except OSError:
        return None
    text = _strip_ansi(raw)

    passed_total = 0
    failed_total = 0
    count_total = 0
    saw_summary = False
    for match in _SUMMARY_RE.finditer(text):
        saw_summary = True
        pct, failed, total = (int(g) for g in match.groups())
        # Derive passed from the out-of total minus failures (robust to the
        # rounded percentage, which can disagree by one on edge cases).
        passed = total - failed
        passed_total += passed
        failed_total += failed
        count_total += total

    if saw_summary:
        # total here is the union of all tier runs (passed + failed).
        return passed_total, passed_total + failed_total

    # Fall back to per-test markers in LastTest.log. Each marker is one test.
    passed = len(re.findall(r'^Test Passed\.', text, re.MULTILINE))
    failed = len(re.findall(r'^Test Failed\.', text, re.MULTILINE))
    if passed == 0 and failed == 0:
        return None
    return passed, passed + failed


# ---------------------------------------------------------------------------
# Coverage
# ---------------------------------------------------------------------------
def _coverage_colour(pct):
    """Return the ANSI colour for a coverage percentage (matches coverage_block)."""
    if pct >= 80:
        return GREEN
    if pct >= 60:
        return CYAN
    if pct >= 40:
        return YELLOW
    if pct > 0:
        return ORANGE
    return RED


def _measures_coverage(path):
    """Read (covered, total, pct) from a cached sonar-measures.json, or None.

    The measures JSON is the SAME number coverage_block.py shows as the live
    SonarCloud headline (cached by the sonar-summary target). Reads
    ``coverage`` + ``lines_to_cover`` + ``uncovered_lines`` so the covered/
    total line matches the headline exactly.
    """
    if not path or not os.path.isfile(path):
        return None
    try:
        with open(path) as handle:
            data = json.load(handle)
    except (OSError, ValueError):
        return None
    measures = {}
    for m in (data.get('component', {}) or {}).get('measures', []) or []:
        if m.get('metric'):
            measures[m['metric']] = m.get('value')
    if 'coverage' not in measures:
        return None
    try:
        pct = float(measures['coverage'])
    except (TypeError, ValueError):
        return None
    try:
        total = int(float(measures.get('lines_to_cover', 0) or 0))
        uncovered = int(float(measures.get('uncovered_lines', 0) or 0))
        covered = total - uncovered
    except (TypeError, ValueError):
        covered, total = None, None
    return covered, total, pct


def _lcov_coverage(path):
    """Aggregate (covered, total, pct) over /src/ lcov records (fallback: all)."""
    if not path or not os.path.isfile(path):
        return None
    records = []
    current = None
    lf = 0
    lh = 0
    try:
        with open(path) as handle:
            for line in handle:
                line = line.rstrip('\n')
                if line.startswith('SF:'):
                    current = line[3:]
                    lf = 0
                    lh = 0
                elif line.startswith('LF:'):
                    try:
                        lf = int(line[3:])
                    except ValueError:
                        lf = 0
                elif line.startswith('LH:'):
                    try:
                        lh = int(line[3:])
                    except ValueError:
                        lh = 0
                elif line.startswith('end_of_record') and current is not None:
                    records.append((current, lf, lh))
                    current = None
    except OSError:
        return None
    if not records:
        return None
    src = [r for r in records if '/src/' in r[0]]
    scope = src if src else records
    total = sum(r[1] for r in scope)
    hit = sum(r[2] for r in scope)
    pct = (100.0 * hit / total) if total else 0.0
    return hit, total, pct


def _xccov_coverage(path):
    """Read (covered, total, pct) from an xccov report JSON."""
    if not path or not os.path.isfile(path):
        return None
    try:
        with open(path) as handle:
            data = json.load(handle)
    except (OSError, ValueError):
        return None
    covered = data.get('coveredLines')
    executable = data.get('executableLines')
    if not isinstance(covered, (int, float)) or not isinstance(executable, (int, float)):
        return None
    pct = (100.0 * covered / executable) if executable else 0.0
    return int(covered), int(executable), pct


def local_coverage(path, local_type):
    """Return (covered, total, pct) for the local source, or None if unavailable."""
    if not path or local_type == 'none':
        return None
    if local_type == 'lcov':
        return _lcov_coverage(path)
    if local_type == 'xccov':
        return _xccov_coverage(path)
    return None


def coverage_for(cov_measures_path, local_cov_path, local_type):
    """Prefer cached SonarCloud measures (the displayed headline), else local.

    Mirrors coverage_block.py's precedence: the live SonarCloud number is the
    headline; local is the secondary source. Here we use the CACHED measures
    JSON (fast, no live re-query) when present, else fall back to local lcov/
    xccov so the field is honest even before a scan/token exists.
    """
    cov = _measures_coverage(cov_measures_path)
    if cov is not None:
        return cov
    return local_coverage(local_cov_path, local_type)


# ---------------------------------------------------------------------------
# Sonar issues
# ---------------------------------------------------------------------------
def _sonar_colour(has_blocker, open_count):
    """RED if any BLOCKER severity, else YELLOW by open count (keep readable)."""
    if has_blocker:
        return RED
    if open_count > 0:
        return YELLOW
    return GREEN


def _impact_severity_facet(data):
    """Return the ``impactSeverities`` facet as a {severity: count} dict, or None.

    Mirrors sonar_summary.py.impact_severity_facet: reading the API's own
    server-side facet (instead of re-deriving it per issue) keeps local counts
    identical to the dashboard severity widget.
    """
    for facet in data.get('facets') or []:
        if facet.get('property') == 'impactSeverities':
            return {v.get('val'): v.get('count', 0)
                    for v in facet.get('values') or []}
    return None


def _removed_count(removed_facet_path):
    """Return the REMOVED issue count from a cached removed-facet report, or 0.

    The removed-facet report is the ``resolutions=REMOVED&facets=
    impactSeverities`` response the sonar-summary target curls separately. Its
    ``impactSeverities`` facet sums to the REMOVED issue count (mirrors
    sonar_summary.py, where ``removed_total = sum(removed_facet.values())``).
    Returns 0 when the file is absent, unreadable, or carries no facet -- so
    callers always get a sane ``total = open + removed`` (open-only when no
    removed data exists, never a crash).
    """
    if not removed_facet_path or not os.path.isfile(removed_facet_path):
        return 0
    try:
        with open(removed_facet_path) as handle:
            data = json.load(handle)
    except (OSError, ValueError):
        return 0
    facet = _impact_severity_facet(data)
    return sum(facet.values()) if facet else 0


def parse_sonar(report_path, removed_facet_path=None):
    """Return (open, total, has_blocker, removed) from cached reports, or None.

    ``open`` + ``has_blocker`` come from the OPEN report's ``impactSeverities``
    facet when present (the dashboard's own server-side count), else from the
    per-issue impacts/legacy severity field. ``total`` is ``open + removed``
    -- the SAME OPEN union REMOVED definition the SonarCloud dashboard and the
    detailed sonar_summary.py use. ``removed`` is read from the separate
    removed-facet report (``removed_facet_path``); when that is absent/empty
    ``removed`` is 0 and ``total`` falls back to ``open``.

    The returned tuple carries ``removed`` (and an implied open-only flag when
    it is 0 only because the file was missing) so the emitter can annotate a
    fallback. To distinguish "0 removed genuinely" from "no removed-facet
    file", the 4th element is ``None`` when the file was absent/unparseable
    and an ``int`` (possibly 0) when it was read.
    """
    if not report_path or not os.path.isfile(report_path):
        return None
    try:
        with open(report_path) as handle:
            data = json.load(handle)
    except (OSError, ValueError):
        return None

    # Prefer the impactSeverities facet (dashboard's own server-side count).
    facet = _impact_severity_facet(data)
    if facet is not None:
        open_count = sum(facet.values())
        has_blocker = facet.get('BLOCKER', 0) > 0
    else:
        # Fall back to per-issue counting over the OPEN issues in the report.
        issues = data.get('issues') or []
        open_count = len(issues)
        has_blocker = False
        for issue in issues:
            # Modern impacts taxonomy (preferred).
            impacts = issue.get('impacts') or []
            for imp in impacts:
                if isinstance(imp, dict) and imp.get('severity') == 'BLOCKER':
                    has_blocker = True
                    break
            if has_blocker:
                break
            # Legacy severity field.
            if issue.get('severity') == 'BLOCKER':
                has_blocker = True
                break

    # REMOVED count from the separate removed-facet report. None => the file
    # was absent/unparseable (open-only fallback); int => genuinely read.
    removed_present = bool(removed_facet_path) and os.path.isfile(
        removed_facet_path or '')
    removed = _removed_count(removed_facet_path) if removed_present else None
    effective_removed = removed if removed is not None else 0
    total = open_count + effective_removed
    return open_count, total, has_blocker, removed


# ---------------------------------------------------------------------------
# Line emission
# ---------------------------------------------------------------------------
def emit_line(label, tests, cov, sonar):
    """Print the single coloured headline line for one repo.

    ``tests`` is (passed, total) or None; ``cov`` is (covered, total, pct) or
    None; ``sonar`` is (open, total, has_blocker, removed) or None. Missing
    fields are OMITTED so the line never prints garbage. When ``removed`` is
    None the total fell back to open-only (no removed-facet file); a GREY
    ``(open-only)`` marker annotates that so a redundant-looking
    ``open X / total X`` is never mistaken for the merged OPEN union REMOVED
    total.
    """
    parts = []

    if tests is not None:
        passed, total = tests
        failed = total - passed
        if failed > 0:
            tests_str = '{}tests: FAIL ({}/{} passed, {} failed){}'.format(
                RED, passed, total, failed, RESET)
        else:
            tests_str = '{}tests: {}/{}{}'.format(GREEN, passed, total, RESET)
        parts.append(tests_str)

    if cov is not None:
        covered, total, pct = cov
        colour = _coverage_colour(pct)
        if covered is not None and total:
            cov_str = '{}cov: {:.1f}%{} {}{}/{}{}'.format(
                colour, pct, RESET, GREY, covered, total, RESET)
        else:
            cov_str = '{}cov: {:.1f}%{}'.format(colour, pct, RESET)
        parts.append(cov_str)

    if sonar is not None:
        open_count, total, has_blocker, removed = sonar
        colour = _sonar_colour(has_blocker, open_count)
        # Annotate when the total is open-only (no removed-facet file). When
        # removed is an int (incl. 0) the merged total is genuine and no marker
        # is needed -- open==total with 0 removed is correct, not redundant.
        # The marker is emitted OUTSIDE the coloured span (own GREY + RESET) so
        # it reads as a neutral note regardless of the sonar colour.
        if removed is None:
            sonar_str = '{}sonar: open {} / total {}{} {}(open-only){}'.format(
                colour, open_count, total, RESET, GREY, RESET)
        else:
            sonar_str = '{}sonar: open {} / total {}{}'.format(
                colour, open_count, total, RESET)
        parts.append(sonar_str)

    body = ' {}|{} '.format(GREY, RESET).join(parts) if parts else \
        '{}(no summary data){}'.format(GREY, RESET)
    print('{}{}{} {}{}'.format(BOLD, label, RESET, body, ''))


def main(argv=None):
    """Parse args and emit one headline line. Always exits 0 (display helper)."""
    p = argparse.ArgumentParser(
        description='Emit a compact end-of-make headline line for one repo.')
    p.add_argument('--label', required=True,
                   help='Repo label, e.g. "[engine-sim-bridge]"')
    p.add_argument('--test-log',
                   help='ctest log with N%% tests passed summary or per-test markers')
    p.add_argument('--cov-measures',
                   help='Cached sonar-measures.json (preferred coverage source)')
    p.add_argument('--local-cov',
                   help='Local coverage file (lcov.info or xccov JSON)')
    p.add_argument('--local-type', choices=('lcov', 'xccov', 'none'),
                   default='none',
                   help='Local coverage source type (default: none)')
    p.add_argument('--sonar-report',
                   help='Cached sonar-report.json (issues/search response)')
    p.add_argument('--removed-facet',
                   help='Cached resolutions=REMOVED&facets=impactSeverities report '
                        '(makes total = open + removed, matching sonar_summary.py)')
    args = p.parse_args(argv)

    try:
        tests = parse_tests(args.test_log)
        cov = coverage_for(args.cov_measures, args.local_cov, args.local_type)
        sonar = parse_sonar(args.sonar_report, args.removed_facet)
        emit_line(args.label, tests, cov, sonar)
    except Exception as exc:  # never crash a display target
        print('{}{} summary failed: {}{}'.format(RED, args.label, exc, RESET),
              file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
