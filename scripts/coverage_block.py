#!/usr/bin/env python3
"""Emit the shared multi-line coverage summary block.

This is the BLOCK-EMISSION helper shared by the engine-sim-cli and engine-sim-app
``coverage-summary`` Makefile targets. It mirrors the visual block the bridge's
own ``coverage_summary.py`` prints (Overall / source / local / source /
exclusions applied), but contains NONE of the bridge-specific extras (top-5
worst, dead-stripped detection, ESP32/platform guards). Those belong to the
bridge; this helper is intentionally minimal and parameterized.

Block format (matches the bridge):

    Overall Coverage (SonarCloud): X%  covered/total
    source: live https://sonarcloud.io /api/measures/component
    local <type>: Y%  covered/total lines
    source: <local coverage file>
    exclusions applied (sonar scope): <sonar-project.properties>

Coloring matches the bridge exactly: BOLD + threshold color on the percentage
(green >=80 / cyan >=60 / yellow >=40 / orange >0 / red =0), grey on the rest.
The "local <type>" line is OMITTED when ``--local-type none`` (or no local
coverage path is given) — callers without a local source (currently none, but
kept honest) must not fabricate a local number.

Local coverage sources supported:

    lcov : an lcov.info file (llvm-cov export). Aggregated over /src/ SF
           records, the same scope rule the bridge uses. Falls back to ALL
           records when no /src/ record is present.
    xccov: an ``xcrun xccov view --report --json`` JSON file. Uses the
           top-level coveredLines/executableLines aggregates.
    none : no local source; the local line is omitted.

SonarCloud-live GET mirrors the bridge: reads SONAR_TOKEN_ES / SONAR_TOKEN and
calls ``/api/measures/component?metricKeys=coverage,lines_to_cover,
uncovered_lines`` for the given project key. Falls back gracefully (local only
+ a "sonar unavailable" note) when there is no token or the fetch/parse fails.

Usage:

    coverage_block.py --project-key KEY \\
        [--local-cov PATH --local-type lcov|xccov] \\
        [--exclusions PATH] [--label TEXT]

Exit codes: 0 always (a missing token / fetch failure / absent local file is
reported in-line, never a crash — this is a display helper, not a build step).
"""
import argparse
import json
import os
import sys
import urllib.parse
import urllib.request

RED = '\033[31m'
ORANGE = '\033[38;5;208m'
YELLOW = '\033[33m'
GREEN = '\033[32m'
CYAN = '\033[36m'
GREY = '\033[90m'
BOLD = '\033[1m'
RESET = '\033[0m'

SONAR_HOST = 'https://sonarcloud.io'


def coverage_color(pct):
    """Return the ANSI color for a coverage percentage (matches the bridge)."""
    if pct >= 80:
        return GREEN
    if pct >= 60:
        return CYAN
    if pct >= 40:
        return YELLOW
    if pct > 0:
        return ORANGE
    return RED


def fetch_sonar_coverage(project_key):
    """Live GET the SonarCloud coverage measures for the headline.

    Reads SONAR_TOKEN_ES / SONAR_TOKEN and calls
    ``/api/measures/component?metricKeys=coverage,lines_to_cover,
    uncovered_lines`` for ``project_key``. Returns a dict of measures on
    success, or None when there is no token or the fetch/parse fails (caller
    falls back to local-only).
    """
    token = os.environ.get('SONAR_TOKEN_ES') or os.environ.get('SONAR_TOKEN')
    if not token:
        return None
    query = urllib.parse.urlencode({
        'component': project_key,
        'metricKeys': 'coverage,lines_to_cover,uncovered_lines',
    })
    url = f'{SONAR_HOST}/api/measures/component?{query}'
    req = urllib.request.Request(url)
    # HTTP Basic auth with the token as the username (empty password) — same
    # auth shape the rest of the pipeline uses with `curl -u "$TOKEN:"`.
    import base64
    cred = base64.b64encode(f'{token}:'.encode()).decode()
    req.add_header('Authorization', f'Basic {cred}')
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            data = json.loads(resp.read().decode('utf-8'))
    except (OSError, ValueError):
        return None
    measures = {}
    for m in (data.get('component', {}) or {}).get('measures', []) or []:
        metric = m.get('metric')
        value = m.get('value')
        if metric and value is not None:
            try:
                measures[metric] = float(value)
            except ValueError:
                measures[metric] = value
    if 'coverage' not in measures:
        return None
    return measures


def parse_lcov(path):
    """Parse an lcov.info file into (filepath, lines_found, lines_hit) records."""
    records = []
    current = None
    lf = 0
    lh = 0
    try:
        with open(path) as f:
            for line in f:
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
        return []
    return records


def _lcov_pct(path):
    """Aggregate (hit, found, pct) over /src/ lcov records (fallback: all)."""
    records = parse_lcov(path)
    if not records:
        return None
    src_records = [r for r in records if '/src/' in r[0]]
    scope = src_records if src_records else records
    found = sum(r[1] for r in scope)
    hit = sum(r[2] for r in scope)
    pct = (100.0 * hit / found) if found else 0.0
    return hit, found, pct


def _xccov_pct(path):
    """Read (covered, executable, pct) from an xccov report JSON."""
    try:
        with open(path) as f:
            data = json.load(f)
    except (OSError, ValueError):
        return None
    covered = data.get('coveredLines')
    executable = data.get('executableLines')
    if not isinstance(covered, (int, float)) or not isinstance(executable, (int, float)):
        return None
    pct = (100.0 * covered / executable) if executable else 0.0
    return int(covered), int(executable), pct


def local_coverage(path, local_type):
    """Return (hit, total, pct) for the local source, or None if unavailable.

    ``local_type`` is one of lcov|xccov|none. ``none`` and a missing/unreadable
    file both yield None so the caller OMITS the local line rather than
    fabricating a number.
    """
    if local_type == 'none' or not path:
        return None
    if not os.path.isfile(path):
        return None
    if local_type == 'lcov':
        return _lcov_pct(path)
    if local_type == 'xccov':
        return _xccov_pct(path)
    return None


def emit_block(project_key, local_cov_path, local_type, exclusions_path, label):
    """Print the shared coverage summary block.

    Headline is the live SonarCloud coverage (matches the dashboard). The
    ``local <type>`` line follows when a local source is available and its
    type is not ``none``. The exclusions line follows when a properties path is
    given. Falls back gracefully: missing token / failed fetch shows the local
    number as headline with a "sonar unavailable" note.
    """
    local = local_coverage(local_cov_path, local_type)

    print('')
    print('=== {} Coverage ==='.format(label))
    sonar = fetch_sonar_coverage(project_key)
    if sonar is not None:
        sc_cov = sonar.get('coverage', 0.0)
        try:
            sc_cov = float(sc_cov)
        except (TypeError, ValueError):
            sc_cov = 0.0
        sc_ltc = int(sonar.get('lines_to_cover', 0) or 0)
        sc_covd = sc_ltc - int(sonar.get('uncovered_lines', 0) or 0)
        print('  {}Overall Coverage (SonarCloud): {}{:.1f}%{}  '
              '{}{}/{}{}'.format(
                  BOLD, coverage_color(sc_cov), sc_cov, RESET,
                  GREY, sc_covd, sc_ltc, RESET))
        print('  {}source: live {} /api/measures/component{}'.format(
            GREY, SONAR_HOST, RESET))
    elif local is not None:
        # No token or fetch failed — promote local to the headline with a note
        # so it is never mistaken for the dashboard number.
        hit, total, pct = local
        print('  {}Overall Coverage (local {}): {}{:.1f}%{}  '
              '{}{}/{} lines{}'.format(
                  BOLD, local_type, coverage_color(pct), pct, RESET,
                  GREY, hit, total, RESET))
        print('  {}source: {}{}'.format(GREY, local_cov_path, RESET))
        print('  {}SonarCloud live unavailable — no token or fetch failed; '
              'showing local only{}'.format(GREY, RESET))
    else:
        print('  {}No coverage data yet (no SonarCloud token and no local '
              'coverage file){}'.format(GREY, RESET))

    if sonar is not None and local is not None:
        hit, total, pct = local
        print('  {}local {}: {}{:.1f}%{}  {}{}/{} lines{}'.format(
            GREY, local_type, coverage_color(pct), pct, RESET,
            GREY, hit, total, RESET))
        print('  {}source: {}{}'.format(GREY, local_cov_path, RESET))

    if exclusions_path:
        print('  {}exclusions applied ({}): {}{}'.format(
            GREY, 'sonar scope', RESET, exclusions_path))
    print('')


def main(argv=None):
    """Parse args and emit the block. Always exits 0 (display helper)."""
    p = argparse.ArgumentParser(
        description='Emit the shared coverage summary block.')
    p.add_argument('--project-key', required=True,
                   help='SonarCloud componentKey (e.g. danieljsinclair_engine-sim-cli)')
    p.add_argument('--local-cov',
                   help='Path to the local coverage file (lcov.info or xccov JSON)')
    p.add_argument('--local-type', choices=('lcov', 'xccov', 'none'),
                   default='none',
                   help='Local coverage source type (default: none -> omit local line)')
    p.add_argument('--exclusions',
                   help='Path to sonar-project.properties (exclusions source)')
    p.add_argument('--label', default='Coverage',
                   help='Block header label')
    args = p.parse_args(argv)

    local_cov_path = args.local_cov if args.local_type != 'none' else None
    try:
        emit_block(args.project_key, local_cov_path, args.local_type,
                   args.exclusions, args.label)
    except Exception as exc:  # never crash a display target
        print('  Failed to emit coverage block: {}'.format(exc), file=sys.stderr)
    return 0


if __name__ == '__main__':
    sys.exit(main())
