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


def parse_sonar_properties(path):
    """Parse sonar-project.properties and return dict of key-value pairs."""
    props = {}
    if not path or not os.path.isfile(path):
        return props
    try:
        with open(path) as f:
            for line in f:
                line = line.strip()
                if line and not line.startswith('#') and '=' in line:
                    key, value = line.split('=', 1)
                    props[key.strip()] = value.strip()
    except OSError:
        pass
    return props


def derive_project_root(local_cov_path, exclusions_path):
    """
    Derive project root from exclusions_path (sonar-project.properties) or local_cov_path.
    Priority: exclusions_path > local_cov_path (build-cov/ or build/).
    """
    if exclusions_path:
        return os.path.dirname(os.path.abspath(exclusions_path))
    if not local_cov_path:
        return None
    abs_path = os.path.abspath(local_cov_path)
    if '/build-cov/' in abs_path:
        return abs_path.split('/build-cov/')[0]
    if '/build/' in abs_path:
        return abs_path.split('/build/')[0]
    return None


def get_coverage_scope(project_root, exclusions_path):
    """
    Return (include_patterns, exclude_patterns) for coverage scope.
    Reads from sonar-project.properties to keep a single source of truth.
    """
    props = parse_sonar_properties(exclusions_path)
    includes = []
    excludes = []

    # Get sources (default to src/)
    sources = props.get('sonar.sources', 'src')
    for s in sources.split(','):
        s = s.strip()
        if s:
            includes.append(os.path.join(project_root, s))

    # Get exclusions (sonar.exclusions + sonar.coverage.exclusions)
    for key in ('sonar.exclusions', 'sonar.coverage.exclusions'):
        val = props.get(key, '')
        for e in val.split(','):
            e = e.strip()
            if e:
                # Convert glob patterns to path prefixes
                # e.g., "engine-sim-bridge/**" -> project_root/engine-sim-bridge/
                # e.g., "build/**" -> project_root/build/
                # e.g., "test/**" -> project_root/test/
                if e.endswith('/**'):
                    prefix = os.path.join(project_root, e[:-3])
                    excludes.append(prefix)
                elif e.endswith('**'):
                    # Handle patterns like "test/**" (no leading slash)
                    prefix = os.path.join(project_root, e[:-2])
                    excludes.append(prefix)
                elif '*' not in e:
                    # Exact directory/file
                    excludes.append(os.path.join(project_root, e))

    return includes, excludes


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
    """Parse an lcov.info into (filepath, lines_found, lines_hit) records.

    ``lines_found``/``lines_hit`` are derived by COUNTING ``DA:`` records (and
    those with a hit count > 0), NOT from the ``LF:``/``LH:`` header integers.
    The headers are emitted by ``geninfo``/llvm-cov and are NOT always equal to
    the actual ``DA:`` record set — they drift by a few lines per file (the
    SonarCloud upload path ``lcov_to_xml.py`` ingests one ``<lineToCover>`` per
    ``DA:``, so summing headers here would diverge from the dashboard). Counting
    DA keeps the local headline on the same basis as the SonarCloud upload, so
    local-vs-Sonar gaps reflect only genuine scope/line-counting differences,
    never an LF/LH artefact. xccov-sourced lcov (engine-sim-app) writes LF/LH
    consistent with its own DA, so this change is a no-op there.
    """
    records = []
    current = None
    da_count = 0
    da_hits = 0
    try:
        with open(path) as f:
            for line in f:
                line = line.rstrip('\n')
                if line.startswith('SF:'):
                    current = line[3:]
                    da_count = 0
                    da_hits = 0
                elif line.startswith('DA:'):
                    counted = _count_da_line(line)
                    if counted is not None:
                        da_count += 1
                        if counted:
                            da_hits += 1
                elif line.startswith('end_of_record') and current is not None:
                    records.append((current, da_count, da_hits))
                    current = None
    except OSError:
        return []
    return records


def _count_da_line(line):
    """Return True if a ``DA:`` line is hit, False if not, None if unparseable.

    A ``DA:`` record is ``DA:<lineNumber>,<executionCount>[,<checksum>]``. The
    single DRY parser used by ``parse_lcov`` to derive per-record found/hit from
    the DA set (rather than trusting the LF/LH headers). ``True``/``False`` mean
    "this line counts as found, and is hit / not hit"; ``None`` means the record
    could not be parsed and is skipped (not counted as found).
    """
    parts = line[3:].split(',')
    if len(parts) < 2:
        return None
    try:
        return int(parts[1]) > 0
    except ValueError:
        return None


def _lcov_pct(path, project_root=None, exclusions_path=None):
    """Aggregate (hit, found, pct) over project src/ lcov records (fallback: all)."""
    records = parse_lcov(path)
    if not records:
        return None
    scope = _filter_lcov_records(records, project_root, exclusions_path)
    return _aggregate_lcov(scope)


def _filter_lcov_records(records, project_root, exclusions_path):
    """Filter lcov records to the coverage scope."""
    if not project_root:
        return _default_src_filter(records)

    if exclusions_path:
        includes, excludes = get_coverage_scope(project_root, exclusions_path)
        return [r for r in records if _in_scope(r[0], includes, excludes, project_root)]

    return _project_src_filter(records, project_root)


def _normalize_sf_path(filepath, project_root):
    """Normalize an lcov SF: path to an absolute form for prefix matching.

    Include/exclude prefixes are always built as absolute paths from
    ``project_root`` (see get_coverage_scope). SF paths, however, differ by
    producer: llvm-cov (CLI) writes ABSOLUTE paths, while Apple's xccov_to_lcov
    (app) writes RELATIVE paths like "EngineSimApp/ContentView.swift" for
    portability into SonarCloud. A relative SF path never ``startswith`` an
    absolute prefix, so resolve relative paths against ``project_root`` here;
    absolute paths are returned unchanged.
    """
    if not filepath:
        return filepath
    if os.path.isabs(filepath):
        return filepath
    if not project_root:
        return filepath
    return os.path.join(project_root, filepath)


def _in_scope(filepath, includes, excludes, project_root=None):
    """Check if filepath matches include patterns and not exclude patterns.

    ``filepath`` is normalized to absolute (via ``project_root``) before
    matching, so both relative (xccov/Apple) and absolute (llvm-cov) SF paths
    compare correctly against the absolute include/exclude prefixes.
    """
    filepath = _normalize_sf_path(filepath, project_root)
    if not any(filepath.startswith(inc) for inc in includes):
        return False
    if any(filepath.startswith(exc) for exc in excludes):
        return False
    return True


def _project_src_filter(records, project_root):
    """Filter to project_root/src/ excluding third-party and tests."""
    prefix = project_root + '/src/'
    return [r for r in records
            if r[0].startswith(prefix)
            and '/.fetchcache/' not in r[0]
            and '/_deps/' not in r[0]
            and '/test/' not in r[0]]


def _default_src_filter(records):
    """Fallback: all /src/ records."""
    scope = [r for r in records if '/src/' in r[0]]
    return scope if scope else records


def _aggregate_lcov(scope):
    """Aggregate hit/found/pct from filtered scope."""
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


def local_coverage(path, local_type, project_root=None, exclusions_path=None):
    """Return (hit, total, pct) for the local source, or None if unavailable.

    ``local_type`` is one of lcov|xccov|none. ``none`` and a missing/unreadable
    file both yield None so the caller OMITS the local line rather than
    fabricating a number.

    ``project_root`` and ``exclusions_path`` are passed through to _lcov_pct
    for scope filtering.
    """
    if local_type == 'none' or not path:
        return None
    if not os.path.isfile(path):
        return None
    if local_type == 'lcov':
        return _lcov_pct(path, project_root, exclusions_path)
    if local_type == 'xccov':
        return _xccov_pct(path)
    return None


def _print_headline_sonar(sc_cov, sc_covd, sc_ltc):
    """Print SonarCloud headline."""
    print('  {}Overall Coverage (SonarCloud): {}{:.1f}%{}  '
          '{}{}/{}{}'.format(
              BOLD, coverage_color(sc_cov), sc_cov, RESET,
              GREY, sc_covd, sc_ltc, RESET))
    print('  {}source: live {} /api/measures/component{}'.format(
        GREY, SONAR_HOST, RESET))


def _print_headline_local(local_type, pct, hit, total, local_cov_path, note):
    """Print local coverage headline with note."""
    print('  {}Overall Coverage (local {}): {}{:.1f}%{}  '
          '{}{}/{} lines{}'.format(
              BOLD, local_type, coverage_color(pct), pct, RESET,
              GREY, hit, total, RESET))
    print('  {}source: {}{}'.format(GREY, local_cov_path, RESET))
    print('  {}{}'.format(GREY, note))


def _print_local_comparison(local_type, pct, hit, total, local_cov_path):
    """Print local coverage comparison line when SonarCloud is also available."""
    print('  {}local {}: {}{:.1f}%{}  {}{}/{} lines{}'.format(
        GREY, local_type, coverage_color(pct), pct, RESET,
        GREY, hit, total, RESET))
    print('  {}source: {}{}'.format(GREY, local_cov_path, RESET))


def _print_exclusions(exclusions_path):
    """Print exclusions applied line."""
    print('  {}exclusions applied ({}): {}{}'.format(
        GREY, 'sonar scope', RESET, exclusions_path))


def emit_block(project_key, local_cov_path, local_type, exclusions_path, label):
    """Print the shared coverage summary block."""
    project_root = derive_project_root(local_cov_path, exclusions_path)
    local = local_coverage(local_cov_path, local_type, project_root, exclusions_path)

    print('')
    print('=== {} Coverage ==='.format(label))
    sonar = fetch_sonar_coverage(project_key)
    if sonar is not None:
        sc_cov = float(sonar.get('coverage', 0) or 0)
        sc_ltc = int(sonar.get('lines_to_cover', 0) or 0)
        sc_covd = sc_ltc - int(sonar.get('uncovered_lines', 0) or 0)

        # Stale SonarCloud (0%) but local has data → promote local
        if sc_cov == 0.0 and local is not None:
            hit, total, pct = local
            _print_headline_local(local_type, pct, hit, total, local_cov_path,
                'SonarCloud headline is stale (0%) — showing local '
                'coverage; dashboard will update once server processes the '
                'latest scan')
        else:
            _print_headline_sonar(sc_cov, sc_covd, sc_ltc)

    elif local is not None:
        # No token or fetch failed → promote local with note
        hit, total, pct = local
        _print_headline_local(local_type, pct, hit, total, local_cov_path,
            'SonarCloud live unavailable — no token or fetch failed; '
            'showing local only')
    else:
        print('  {}No coverage data yet (no SonarCloud token and no local '
              'coverage file){}'.format(GREY, RESET))

    # Local comparison line when both SonarCloud and local available
    if sonar is not None and local is not None:
        hit, total, pct = local
        sc_cov = float(sonar.get('coverage', 0) or 0)
        if sc_cov != 0.0:  # Not stale, show comparison
            _print_local_comparison(local_type, pct, hit, total, local_cov_path)

    if exclusions_path:
        _print_exclusions(exclusions_path)
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
