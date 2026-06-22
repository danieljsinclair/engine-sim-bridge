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

    tests       : GREEN + "PASS" prefix if all passed; RED + "FAIL" prefix if any failed.
    coverage %  : GREEN >=80 / CYAN >=60 / YELLOW >=40 / ORANGE >0 / RED =0.
    sonar       : RED if a BLOCKER is present, YELLOW if open>0 with no blocker,
                  GREEN if 0 open (clean). A short reason suffix (GREY) names
                  the tier: "(N blocker)" / "(no blocker)" / "(clean)".

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


def _xcresult_metric(metrics, key):
    """Read an int metric from an xcresult metrics dict, or None.

    xcresult JSON wraps each value as ``{"_type": {"_name": "Int"},
    "_value": "<n>"}``. Returns the int value or None when absent/unparseable.
    """
    if not isinstance(metrics, dict):
        return None
    node = metrics.get(key)
    if not isinstance(node, dict):
        return None
    try:
        return int(node.get('_value'))
    except (TypeError, ValueError):
        return None


def parse_xcresult_tests(glob_pattern):
    """Return (passed, total) from the NEWEST xcresult matching ``glob_pattern``.

    iOS app tests run via ``xcodebuild test`` (no greppable ctest log). The
    cleanest count source is the action's ResultMetrics in the .xcresult bundle
    xcodebuild writes under ``<derivedData>/Logs/Test/*.xcresult``: it carries
    ``testsCount`` (total) and ``testsFailedCount`` (failures; absent or 0 when
    all pass). The newest bundle is used (a run may leave several over time).
    Returns None when no bundle exists, xcresulttool is unavailable, or the
    metrics are absent so the caller OMITS the tests field gracefully.
    """
    if not glob_pattern:
        return None
    import glob as _glob
    bundles = sorted(_glob.glob(glob_pattern), key=os.path.getmtime)
    if not bundles:
        return None
    xcresult = bundles[-1]
    dump_path = _xcresult_dump_path()
    try:
        # --legacy gives the stable JSON shape with metrics at the top level.
        rc = os.system(
            'xcrun xcresulttool get --legacy --format json --path {} >{}'.format(
                _shell_quote(xcresult), _shell_quote(dump_path)))
        if rc != 0:
            return None
        with open(dump_path, errors='replace') as handle:
            data = json.load(handle)
    except (OSError, ValueError):
        return None
    finally:
        try:
            os.unlink(dump_path)
        except OSError:
            pass
    metrics = data.get('metrics') or {}
    total = _xcresult_metric(metrics, 'testsCount')
    if total is None:
        # Some bundles carry metrics only under actions[].actionResult.metrics.
        for action in (data.get('actions', {}) or {}).get('_values', []) or []:
            am = (action.get('actionResult', {}) or {}).get('metrics', {})
            total = _xcresult_metric(am, 'testsCount')
            if total is not None:
                metrics = am
                break
    if total is None:
        return None
    failed = _xcresult_metric(metrics, 'testsFailedCount') or 0
    return total - failed, total


def _xcresult_dump_path():
    """Return a temp path for the xcresulttool JSON dump (cleaned on exit)."""
    import tempfile
    fd, path = tempfile.mkstemp(prefix='build_summary_xcresult_', suffix='.json')
    os.close(fd)
    return path


def _shell_quote(value):
    """Single-quote ``value`` for safe shell interpolation (sh style)."""
    if not value:
        return "''"
    return "'" + value.replace("'", "'\\''") + "'"


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
def _sonar_colour(blocker_count, open_count):
    """Sonar colour by a 3-tier rule.

    RED    = a BLOCKER severity is present (``blocker_count > 0``).
    YELLOW = open issues but NO blocker.
    GREEN  = 0 open issues (clean).

    The BLOCKER tier dominates: any blocker is RED regardless of the open
    count. Without a blocker the open count decides YELLOW (open) vs GREEN
    (clean) -- previously 0-open with no blocker was wrongly YELLOW.
    """
    if blocker_count:
        return RED
    if open_count > 0:
        return YELLOW
    return GREEN


def _sonar_reason(blocker_count, open_count):
    """Short reason suffix explaining the sonar tier (empty when irrelevant).

    RED    (blocker): ``(N blocker)`` -- names the count driving the tier.
    YELLOW (open)   : ``(no blocker)`` -- clarifies why YELLOW not RED.
    GREEN  (clean)  : ``(clean)``.

    The suffix lets a reader tell at a glance why app is YELLOW (open, no
    blocker) while cli/bridge are RED (blocker present) without cross-refering
    the dashboard.
    """
    if blocker_count:
        return '({} blocker)'.format(blocker_count)
    if open_count > 0:
        return '(no blocker)'
    return '(clean)'


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
    """Return (open, total, blocker_count, removed) from cached reports, or None.

    ``open`` + ``blocker_count`` come from the OPEN report's
    ``impactSeverities`` facet when present (the dashboard's own server-side
    count), else from the per-issue impacts/legacy severity field. ``total``
    is ``open + removed`` -- the SAME OPEN union REMOVED definition the
    SonarCloud dashboard and the detailed sonar_summary.py use. ``removed`` is
    read from the separate removed-facet report (``removed_facet_path``); when
    that is absent/empty ``removed`` is 0 and ``total`` falls back to ``open``.

    ``blocker_count`` is the BLOCKER count (>=0 int), read from the SAME
    ``impactSeverities`` facet (the 'BLOCKER' bucket), so the emitter can show
    a ``(N blocker)`` reason suffix without a second pass. The fallback per-
    issue path counts BLOCKER-tagged issues instead.

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
        blocker_count = facet.get('BLOCKER', 0) or 0
    else:
        # Fall back to per-issue counting over the OPEN issues in the report.
        issues = data.get('issues') or []
        open_count = len(issues)
        blocker_count = 0
        for issue in issues:
            # Modern impacts taxonomy (preferred).
            impacts = issue.get('impacts') or []
            issue_is_blocker = any(
                isinstance(imp, dict) and imp.get('severity') == 'BLOCKER'
                for imp in impacts)
            # Legacy severity field.
            if not issue_is_blocker and issue.get('severity') == 'BLOCKER':
                issue_is_blocker = True
            if issue_is_blocker:
                blocker_count += 1

    # REMOVED count from the separate removed-facet report. None => the file
    # was absent/unparseable (open-only fallback); int => genuinely read.
    removed_present = bool(removed_facet_path) and os.path.isfile(
        removed_facet_path or '')
    removed = _removed_count(removed_facet_path) if removed_present else None
    effective_removed = removed if removed is not None else 0
    total = open_count + effective_removed
    return open_count, total, blocker_count, removed


# ---------------------------------------------------------------------------
# Line emission
# ---------------------------------------------------------------------------

# Fixed column widths so the three repo lines align vertically. ANSI colour
# codes are ZERO display-width but non-zero string length, so padding MUST be
# computed on the VISIBLE text (ANSI-stripped) and appended as PLAIN spaces
# OUTSIDE any colour span -- padding inside a colour run would either paint
# trailing space or misalign when the terminal re-wraps. See ``pad_visible``.
#
# Widths are sized to the widest realistic value in each column so the three
# lines (app/cli/bridge) line up without an over-wide table:
#   label : 19 ("[engine-sim-bridge]")
#   tests : 18 ("tests: PASS 68/68"; FAIL detail form is wider and overflows)
#   cov   : 20 ("cov: 69.3% 3179/4588")
#   sonar : 39 ("sonar: open 378 / total 418 (1 blocker)" + room for
#               "(open-only)" which is the same width as the reason suffix)
COL_WIDTH_LABEL = 19
COL_WIDTH_TESTS = 18
COL_WIDTH_COV = 20
COL_WIDTH_SONAR = 39


def _visible_len(text):
    """Return the display length of ``text`` (ANSI escape codes count as 0).

    Terminals render ANSI colour codes invisibly, so ``len()`` overcounts by the
    combined length of every escape sequence. This strips them first.
    """
    return len(_strip_ansi(text))


def pad_visible(text, width):
    """Right-pad ``text`` with plain spaces so its VISIBLE length is ``width``.

    The pad is appended AFTER the text (outside any ANSI span) as plain spaces,
    so it never inherits a colour and never throws off alignment. When the text
    is already wider than ``width`` it is returned unchanged (no truncation --
    a column never silently drops data).
    """
    pad = width - _visible_len(text)
    return text + (' ' * pad if pad > 0 else '')


def emit_line(label, tests, cov, sonar):
    """Print the single coloured headline line for one repo.

    ``tests`` is (passed, total) or None; ``cov`` is (covered, total, pct) or
    None; ``sonar`` is (open, total, blocker_count, removed) or None. Missing
    fields are OMITTED so the line never prints garbage. When ``removed`` is
    None the total fell back to open-only (no removed-facet file); a GREY
    ``(open-only)`` marker annotates that so a redundant-looking
    ``open X / total X`` is never mistaken for the merged OPEN union REMOVED
    total.

    Each column is padded to a fixed VISIBLE width (see pad_visible) so the
    three repo lines (app, cli, bridge) align vertically -- the labels,
    ``tests:``/``cov:``/``sonar:`` keywords, and ``|`` separators each form a
    column. Padding is plain spaces outside the colour spans.
    """
    parts = []

    if tests is not None:
        passed, total = tests
        failed = total - passed
        if failed > 0:
            tests_str = '{}tests: FAIL ({}/{} passed, {} failed){}'.format(
                RED, passed, total, failed, RESET)
        else:
            tests_str = '{}tests: PASS {}/{}{}'.format(
                GREEN, passed, total, RESET)
        parts.append(pad_visible(tests_str, COL_WIDTH_TESTS))

    if cov is not None:
        covered, total, pct = cov
        colour = _coverage_colour(pct)
        if covered is not None and total:
            cov_str = '{}cov: {:.1f}%{} {}{}/{}{}'.format(
                colour, pct, RESET, GREY, covered, total, RESET)
        else:
            cov_str = '{}cov: {:.1f}%{}'.format(colour, pct, RESET)
        parts.append(pad_visible(cov_str, COL_WIDTH_COV))

    if sonar is not None:
        open_count, total, blocker_count, removed = sonar
        colour = _sonar_colour(blocker_count, open_count)
        reason = _sonar_reason(blocker_count, open_count)
        # Annotate when the total is open-only (no removed-facet file). When
        # removed is an int (incl. 0) the merged total is genuine, so the
        # suffix carries the tier REASON instead. The suffix/marker is emitted
        # OUTSIDE the coloured span (own GREY + RESET) so it reads as a neutral
        # note regardless of the sonar colour.
        if removed is None:
            note = '(open-only)'
        else:
            note = reason
        sonar_str = '{}sonar: open {} / total {}{} {}{}{}'.format(
            colour, open_count, total, RESET, GREY, note, RESET)
        parts.append(pad_visible(sonar_str, COL_WIDTH_SONAR))

    body = ' {}|{} '.format(GREY, RESET).join(parts) if parts else \
        '{}(no summary data){}'.format(GREY, RESET)
    label_str = pad_visible('{}{}{}'.format(BOLD, label, RESET), COL_WIDTH_LABEL)
    print('{} {}'.format(label_str, body))


def main(argv=None):
    """Parse args and emit one headline line. Always exits 0 (display helper)."""
    p = argparse.ArgumentParser(
        description='Emit a compact end-of-make headline line for one repo.')
    p.add_argument('--label', required=True,
                   help='Repo label, e.g. "[engine-sim-bridge]"')
    p.add_argument('--test-log',
                   help='ctest log with N%% tests passed summary or per-test markers')
    p.add_argument('--xcresult-glob',
                   help='Glob for the newest .xcresult bundle (iOS xcodebuild tests). '
                        'Reads ResultMetrics.testsCount/testsFailedCount so the app '
                        'line gets a tests field without a greppable ctest log.')
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
        # Prefer the ctest log when present; fall back to the xcresult bundle
        # (iOS app tests run via xcodebuild, which writes no ctest log).
        tests = parse_tests(args.test_log)
        if tests is None:
            tests = parse_xcresult_tests(args.xcresult_glob)
        cov = coverage_for(args.cov_measures, args.local_cov, args.local_type)
        sonar = parse_sonar(args.sonar_report, args.removed_facet)
        emit_line(args.label, tests, cov, sonar)
    except Exception as exc:  # never crash a display target
        print('{}{} summary failed: {}{}'.format(RED, args.label, exc, RESET),
              file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
