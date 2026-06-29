#!/usr/bin/env python3
"""Display coverage summary: SonarCloud live + local lcov.

Prints TWO coverage numbers so the dashboard value and the honest local value
are both visible (mirroring how the issues section shows open-vs-total):

  - HEADLINE (first line, prominent): the LIVE SonarCloud coverage, fetched at
    display time from /api/measures/component?metricKeys=coverage,
    lines_to_cover,uncovered_lines. This is the number that matches the
    SonarCloud dashboard — same live-GET pattern sonar_summary.py uses for
    issues. Falls back gracefully (local only + "sonar unavailable" note) when
    no token is set or the fetch fails.
  - LOCAL lcov: the honest llvm-cov number over instrumented src files
    (lcov SF records under /src/), clearly labelled "local lcov (excludes
    dead-stripped)" so the small residual gap vs the sonar headline is
    understood, not a mystery.

OUTPUT MODES:
  Default (concise, used by `make`): the two coverage numbers + the TOP 5
  worst (lowest coverage) src files + the MISSING (dead-stripped, 0% on
  SonarCloud) files list + the platform-excluded files list. --verbose /
  --all : prints the full per-file table for every src file.

The earlier "mimic sonar" local heuristic (re-computing the SonarCloud
denominator by adding dead-stripped files at 0%) is retired — now that the
real SonarCloud number is shown live, the approximation was unnecessary and
confusing. The dead-stripped files are still listed (they ARE the 0% files
on the dashboard) but no longer folded into a local headline number.

Two kinds of src file are NOT in the local lcov denominator and are handled
distinctly:

  * PLATFORM-EXCLUDED (e.g. ESP32I2SHardwareProvider.cpp): a src file whose
    ENTIRE body sits behind a host-never-defined platform guard (#ifdef
    ESP_PLATFORM / TARGET_OS_IPHONE / __ANDROID__ ...). The host (macOS)
    preprocessor strips it to an empty translation unit, so it has ZERO
    instrumentable lines, and SonarCloud reports it as "-" (no data, not in
    the denominator). These are EXCLUDED and listed in their own "not
    reported" category — they are NOT coverage gaps.

  * DEAD-STRIPPED: a src file compiled for the host but not linked into any
    test binary (no caller) -> absent from lcov. SonarCloud reports these at
    0.0% (in the denominator, all uncovered). Listed as MISSING below.
"""
import json
import os
import re
import subprocess
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

# SonarCloud component key + host for the live coverage GET. Must match the
# key passed to sonar-scanner (sonar-project.properties: sonar.projectKey).
SONAR_HOST = 'https://sonarcloud.io'
SONAR_COMPONENT = 'danieljsinclair_engine-sim-bridge'


def fetch_sonar_coverage():
    """Live GET the SonarCloud coverage measures for the headline.

    Mirrors sonar_summary.py's live-GET pattern: reads SONAR_TOKEN_ES /
    SONAR_TOKEN from the environment and calls
    /api/measures/component?metricKeys=coverage,lines_to_cover,uncovered_lines.

    Returns a dict {coverage, lines_to_cover, uncovered_lines} (floats/ints)
    on success, or None when there is no token or the fetch/parse fails. A
    None return lets the caller fall back gracefully (show local only, note
    sonar unavailable) rather than crash on a network blip or missing token.
    """
    token = os.environ.get('SONAR_TOKEN_ES') or os.environ.get('SONAR_TOKEN')
    if not token:
        return None
    query = urllib.parse.urlencode({
        'component': SONAR_COMPONENT,
        'metricKeys': 'coverage,lines_to_cover,uncovered_lines',
    })
    url = f'{SONAR_HOST}/api/measures/component?{query}'
    req = urllib.request.Request(url)
    # HTTP Basic auth with the token as the username (empty password) — the
    # same auth shape the rest of the pipeline uses with `curl -u "$TOKEN:"`.
    import base64
    cred = base64.b64encode(f'{token}:'.encode()).decode()
    req.add_header('Authorization', f'Basic {cred}')
    try:
        with urllib.request.urlopen(req, timeout=10) as resp:
            data = json.loads(resp.read().decode('utf-8'))
    except (OSError, ValueError, subprocess.SubprocessError):
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


def coverage_color(pct):
    """Return ANSI color for a coverage percentage."""
    if pct >= 80:
        return GREEN
    if pct >= 60:
        return CYAN
    if pct >= 40:
        return YELLOW
    if pct > 0:
        return ORANGE
    return RED


def parse_lcov(path):
    """Parse an lcov.info file.

    Returns a list of (filepath, lines_found, lines_hit) tuples, in the
    order the records appear. Missing LF/LH default to 0.
    """
    records = []
    current = None
    lf = 0
    lh = 0
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
    return records


def summarize(records):
    """Aggregate (hit, found, pct) over a list of (path, lf, lh) records."""
    found = sum(r[1] for r in records)
    hit = sum(r[2] for r in records)
    pct = (100.0 * hit / found) if found else 0.0
    return hit, found, pct


def is_src(path):
    """True if path is under a /src/ directory (the production source root)."""
    return '/src/' in path


def _glob_to_regex(glob):
    """Convert an Ant-style Sonar glob to a regex.

    Sonar/Ant patterns:
      **  matches zero or more directories
      *   matches anything except a path separator
      ?   matches a single char except a separator
    Anchored at both ends against the repo-root-relative path.
    """
    i = 0
    out = ['^']
    while i < len(glob):
        c = glob[i]
        if glob[i:i + 2] == '**':
            # '**/' matches any number of dirs (including none); '**' alone
            # matches anything including separators.
            if glob[i + 2:i + 3] == '/':
                out.append('(?:.*/)?')
                i += 3
            else:
                out.append('.*')
                i += 2
        elif c == '*':
            out.append('[^/]*')
            i += 1
        elif c == '?':
            out.append('[^/]')
            i += 1
        else:
            out.append(re.escape(c))
            i += 1
    out.append('$')
    return re.compile(''.join(out))


def load_sonar_exclusions(repo_root):
    """Return (compiled_patterns, props_path) from sonar-project.properties.

    Parses sonar.exclusions + sonar.coverage.exclusions (comma separated,
    Ant-style globs). Returns ([], None) if the file is absent so callers
    gracefully fall back to the unfiltered src/ number.
    """
    props_path = os.path.join(repo_root, 'sonar-project.properties')
    patterns = []
    if not os.path.isfile(props_path):
        return [], None
    keys = ('sonar.exclusions', 'sonar.coverage.exclusions')
    with open(props_path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith('#') or '=' not in line:
                continue
            key, _, value = line.partition('=')
            key = key.strip()
            if key not in keys:
                continue
            for raw in value.split(','):
                pat = raw.strip()
                if pat:
                    patterns.append(_glob_to_regex(pat))
    return patterns, props_path


def matches_exclusions(abs_path, repo_root, patterns):
    """True if abs_path matches any exclusion pattern.

    Patterns are matched against the repo-root-relative path (forward slashes),
    mirroring how Sonar resolves sonar.exclusions relative to the project root.
    A pattern with no leading dir (e.g. 'test/**') also matches a path whose
    relative form begins anywhere under that — but Ant globs are already
    root-anchored, so we test the relative path directly.
    """
    if not patterns:
        return False
    rel = os.path.relpath(abs_path, repo_root) if repo_root else abs_path
    rel = rel.replace(os.sep, '/')
    return any(p.search(rel) for p in patterns)


def find_src_files_on_disk(lcov_src_files, repo_root):
    """Return src/*.cpp files on disk that are ABSENT from lcov (dead-stripped).

    These are the files that have no coverage mapping at all — they appear as
    0% on SonarCloud because Sonar counts them against sonar.sources=src.
    Returns (absent_paths, scanned_ok): scanned_ok is False if the src dir
    could not be found, in which case absent_paths is empty.
    """
    src_dir = os.path.join(repo_root, 'src')
    if not os.path.isdir(src_dir):
        return [], False
    on_disk = set()
    for dirpath, _dirs, files in os.walk(src_dir):
        for name in files:
            if name.endswith(('.cpp', '.cc', '.c', '.cxx')):
                abs_path = os.path.abspath(os.path.join(dirpath, name))
                on_disk.add(abs_path)
    lcov_set = set(lcov_src_files)
    absent = sorted(on_disk - lcov_set)
    return absent, True


def count_code_lines(path):
    """Approximate EXECUTABLE line count for a source file with no lcov data.

    Counts non-blank, non-comment lines that a coverage tool (llvm-cov /
    SonarQube) would treat as executable, excluding pure declarations and
    structural-only lines. This is an APPROXIMATION of the LF/lines_to_cover a
    coverage tool would record — it cannot be exact without coverage mapping
    (tools differ on edge cases like lambda captures, defaulted methods, single
    line getters), but it lets the sonar-scope denominator include dead-
    stripped files at 0% so the local number tracks SonarCloud (which counts
    them as uncovered). Returns 0 on read failure (file treated as no
    contribution).

    Excluded (not executable): comments, blank lines, bare braces/semicolons,
    preprocessor directives, pure declarations (ending in ';' with no control-
    flow keyword and no '{'), and function/class signature lines that open a
    block on the next line.
    """
    count = 0
    in_block = False
    # Tokens that, when present, make a ';' line an executable statement
    # (assignment, call, return, throw) rather than a pure declaration.
    exec_tokens = ('=', 'return', 'throw', 'break', 'continue', '(', '<<', '>>')
    control_kw = ('if', 'else', 'for', 'while', 'switch', 'case', 'default',
                  'do', 'goto', 'return', 'throw', 'try', 'catch')
    try:
        with open(path) as f:
            for raw in f:
                line = raw.strip()
                if not line:
                    continue
                # Strip block comments across lines.
                if in_block:
                    if '*/' in line:
                        in_block = False
                        line = line.split('*/', 1)[1].strip()
                    else:
                        continue
                # Remove full-line block comments and trailing // comments.
                if line.startswith('/*'):
                    rest = line[2:]
                    if '*/' in rest:
                        line = rest.split('*/', 1)[1].strip()
                    else:
                        in_block = True
                        continue
                line = line.split('//', 1)[0].strip()
                if not line:
                    continue
                # Preprocessor directives are not executable.
                if line.startswith('#'):
                    continue
                # Bare structural tokens llvm-cov wouldn't count.
                if line in ('{', '}', '};', '};', ';'):
                    continue
                # A line opening a block (signature / class / namespace / ctor
                # init list) is structural, not executable on its own.
                if line.endswith(':') and not any(
                        line.startswith(k + ' ') or line.startswith(k + '(')
                        for k in ('case', 'default', 'public', 'private',
                                  'protected')):
                    # label-style; skip
                    continue
                if line.endswith('{') and not any(
                        line.startswith(k) for k in control_kw):
                    # function/class/struct/enum/namespace signature opening a
                    # body — not itself an executable line.
                    continue
                # Pure declaration: ends in ';' but has no assignment/call/
                # control keyword. Skips `int x;`, `Foo bar;`, `using ...;`,
                # typedefs, forward declares, friend decls, etc.
                if line.endswith(';') and not any(t in line for t in exec_tokens):
                    continue
                count += 1
    except OSError:
        return 0
    return count


# Platform guards that are NEVER defined on the host (macOS) coverage build.
# A src file whose NON-TRIVIAL body (executable lines outside comments/blank)
# sits entirely inside one of these #ifdef blocks compiles to an empty
# translation unit on the host -> zero instrumentable lines -> SonarCloud
# reports it as "-" (no data). AudioHardwareProviderFactory.cpp is the
# counter-example: it has a #elif ESP_PLATFORM branch but ALSO host-active
# code, so it is NOT platform-excluded (it shows 100% on SonarCloud).
HOST_PLATFORM_GUARDS = (
    'ESP_PLATFORM',
    'TARGET_OS_IPHONE',
    '__IPHONE_OS_VERSION_MIN_REQUIRED',
    '__ANDROID__',
    '__linux__',
)


def is_platform_excluded(path):
    """True if a src file's body is entirely behind a host-absent platform guard.

    Detects the ESP32I2SHardwareProvider pattern: the whole file is wrapped in
    `#ifdef ESP_PLATFORM` ... `#endif`, so on the macOS host it preprocesses to
    nothing (zero instrumentable lines) and SonarCloud lists it as "-" rather
    than 0.0%. Such files are categorically NOT coverage gaps — they are code
    for another platform — so they are excluded from the headline denominator
    and shown in their own "not reported" category.

    A file with ANY host-active (unguarded) code is NOT platform-excluded even
    if it also has a guarded branch (e.g. AudioHardwareProviderFactory.cpp).
    """
    try:
        with open(path) as f:
            lines = f.readlines()
    except OSError:
        return False
    # depth of nested host-absent guards currently open at each source line.
    depth = 0
    has_guarded_body = False
    has_host_body = False
    for raw in lines:
        line = raw.strip()
        if not line or line.startswith('//'):
            continue
        stripped = line.split('//', 1)[0].strip()
        if stripped.startswith('#'):
            directive = stripped[1:].strip()
            if directive.startswith('ifdef'):
                guard = directive[5:].strip().split()[0] if len(directive) > 5 else ''
                if guard in HOST_PLATFORM_GUARDS:
                    depth += 1
            elif directive.startswith('ifndef'):
                guard = directive[6:].strip().split()[0] if len(directive) > 6 else ''
                # ifndef of a host-defined macro (rare) is not handled; skip.
                pass
            elif directive.startswith('if '):
                # `#if defined(ESP_PLATFORM)` style.
                if any(g in directive for g in HOST_PLATFORM_GUARDS):
                    depth += 1
            elif directive.startswith('else'):
                # An #else flips host-activeness for the simplest 1-deep case;
                # conservatively treat any #else region as host-active.
                depth = 0
            elif directive.startswith('endif'):
                depth = max(0, depth - 1)
            continue
        # A code line (not a directive, not blank/comment).
        if line in ('{', '}', '};'):
            continue
        if depth > 0:
            has_guarded_body = True
        else:
            has_host_body = True
    return has_guarded_body and not has_host_body


def display(coverage_path, repo_root, verbose=False, label='Local Coverage'):
    """Print the coverage summary.

    Prints TWO coverage numbers so the dashboard value and the honest local
    value are both visible (mirroring how the issues section shows open-vs-
    total). ``label`` names the report section (defaults to a plain header;
    the Makefile passes a repo tag so coverage reads as part of the same
    measurement report as the SonarCloud summary).

    Default (concise, used by `make`): the live SonarCloud headline + the
    local lcov number + the TOP 5 worst (lowest coverage) src files + the
    MISSING (dead-stripped, 0% on SonarCloud) files list + the platform-
    excluded files list. --verbose / --all : prints the full per-file table.
    """
    records = parse_lcov(coverage_path)

    all_hit, all_found, all_pct = summarize(records)
    src_records = [r for r in records if is_src(r[0])]
    src_hit, src_found, src_pct = summarize(src_records)

    # Sonar scope: src/** minus sonar.exclusions + sonar.coverage.exclusions.
    # Used to find the dead-stripped files (the 0% set on SonarCloud).
    patterns, props_path = load_sonar_exclusions(repo_root)
    if patterns:
        sonar_records = [r for r in src_records
                         if not matches_exclusions(r[0], repo_root, patterns)]
    else:
        sonar_records = src_records

    # Absent src files (on disk but not in lcov) split into two categories
    # that SonarCloud reports DIFFERENTLY:
    #   * PLATFORM-EXCLUDED: whole body behind a host-absent guard (#ifdef
    #     ESP_PLATFORM ...) -> empty TU on host -> SonarCloud shows "-" (no
    #     data, NOT in the denominator). Listed in their own "not reported"
    #     category; NOT a coverage gap.
    #   * DEAD-STRIPPED: compiled for host but linked into no test binary ->
    #     SonarCloud shows 0.0% (IN the denominator, all uncovered). Listed
    #     as MISSING below (they ARE the dashboard's 0% set).
    absent, scanned = find_src_files_on_disk(
        [r[0] for r in sonar_records], repo_root)
    platform_excluded = []
    dead_absent = []
    if scanned:
        for path in absent:
            if is_platform_excluded(path):
                platform_excluded.append(path)
            else:
                dead_absent.append(path)

    dead_paths = set(dead_absent)

    # HEADLINE: live SonarCloud coverage (the dashboard number). Fetched at
    # display time — same live-GET pattern sonar_summary.py uses for issues.
    # Falls back gracefully (local only) when there is no token or the fetch
    # fails, so a missing token / network blip never crashes the report.
    sonar_measures = fetch_sonar_coverage()

    print('')
    print('=== {} Coverage ==='.format(label))
    if sonar_measures is not None:
        sc_cov = sonar_measures.get('coverage', 0.0)
        sc_ltc = int(sonar_measures.get('lines_to_cover', 0) or 0)
        sc_covd = sc_ltc - int(sonar_measures.get('uncovered_lines', 0) or 0)
        # HEADLINE first, prominent. This is the number that matches the
        # dashboard. Mirrors how the issues section leads with Open issues.
        print('  {}Overall Coverage (SonarCloud): {}{:.1f}%{}  '
              '{}{}/{}{}'.format(
                  BOLD, coverage_color(sc_cov), sc_cov, RESET,
                  GREY, sc_covd, sc_ltc, RESET))
        print('  {}source: live {} /api/measures/component{}'.format(
            GREY, SONAR_HOST, RESET))
    else:
        # No token or fetch failed — show local as headline with a note so it
        # is never mistaken for the dashboard number.
        print('  {}Overall Coverage (local lcov): {}{:.1f}%{}  '
              '{}{}/{} lines{}'.format(
                  BOLD, coverage_color(src_pct), src_pct, RESET,
                  GREY, src_hit, src_found, RESET))
        print('  {}source: {}{}'.format(GREY, coverage_path, RESET))
        print('  {}SonarCloud live unavailable — no token or fetch failed; '
              'showing local only{}'.format(GREY, RESET))

    # LOCAL lcov: the honest llvm-cov number over instrumented src files.
    # Labelled "excludes dead-stripped" so the residual gap vs the sonar
    # headline (sonar counts the dead-stripped 0% files in its denominator;
    # lcov does not) is understood, not a mystery.
    if sonar_measures is not None:
        print('  {}local lcov (excludes dead-stripped): {}{:.1f}%{}  '
              '{}{}/{} lines{}'.format(
                  GREY, coverage_color(src_pct), src_pct, RESET,
                  GREY, src_hit, src_found, RESET))
        print('  {}source: {}{}'.format(GREY, coverage_path, RESET))
    print('')
    if patterns:
        print('  {}exclusions applied ({}): {}{}'.format(
            GREY, 'sonar scope', RESET, props_path))
        print('')

    # Per-file src coverage, lowest-first. CONCISE by default (top 5 worst);
    # --verbose shows the full table. Ranks only LIVE records (present in lcov
    # WITH executable lines): dead-stripped files (absent from lcov) and files
    # with 0 executable lines are excluded here so they appear ONLY in the
    # MISSING list below, not duplicated in the top-5.
    live_records = [r for r in sonar_records
                    if r[0] not in dead_paths and r[1] > 0]
    live_records.sort(key=lambda r: (r[2] / r[1] if r[1] else 0.0))
    if verbose:
        print('  src/ coverage per-file (sonar scope, lowest first):')
    else:
        print('  src/ coverage — top 5 worst (sonar scope, live only): {}pass --verbose for full list{}'.format(
            GREY, RESET))
    shown = live_records if verbose else live_records[:5]
    for path, lf, lh in shown:
        rel = os.path.relpath(path, repo_root) if repo_root else path
        pct = (100.0 * lh / lf) if lf else 0.0
        marker = '  {}DEAD{} '.format(RED, RESET) if path in dead_paths else ''
        print('    {}{:5.1f}%{:8} {}/{}  {}{}{}'.format(
            coverage_color(pct), pct, RESET, lh, lf, marker, rel, ''))
    print('')

    # Dead-stripped src files: on disk but absent from lcov, NOT platform-
    # excluded. These are the real 0% files on SonarCloud (the dashboard's
    # 0.0% set) — genuine coverage gaps, not a measurement artefact.
    if scanned:
        print('  src files at 0% on SonarCloud (dead-stripped — absent from lcov):')
        if dead_absent:
            for path in dead_absent:
                rel = os.path.relpath(path, repo_root) if repo_root else path
                print('    {}MISSING{}  {}'.format(RED, RESET, rel))
        else:
            print('    {}none — every non-platform src/*.cpp has coverage data{}'.format(
                GREEN, RESET))
        print('')

        # Platform-excluded src files: whole body behind a host-absent guard.
        # SonarCloud reports these as "-" (no data) — they are NOT in the
        # denominator and are NOT coverage gaps, just code for another platform.
        if platform_excluded:
            print('  src files not reported to SonarCloud '
                  '(platform-conditional — no host coverage data):')
            for path in platform_excluded:
                rel = os.path.relpath(path, repo_root) if repo_root else path
                print('    {}EXCLUDED{} {}'.format(GREY, RESET, rel))
            print('')

    print('  ' + BOLD + 'Note:' + RESET +
          ' SonarCloud headline = live dashboard number; local lcov is the')
    print('  honest llvm-cov figure over instrumented src (excludes dead-')
    print('  stripped files, which SonarCloud counts as 0%). The small residual')
    print('  gap is expected: llvm-cov and SonarQube also count executable lines')
    print('  slightly differently per file. Platform-conditional code (e.g. ESP32')
    print('  behind #ifdef ESP_PLATFORM) is not reported by SonarCloud ("-").')
    print('')


def main():
    """Entry point. Accepts optional --verbose/--all and --label flags in argv.

    Usage: coverage_summary.py <lcov.info> [repo_root] [--verbose|--all] [--label NAME]
    The Makefile calls this with just the lcov path (concise output) plus a
    --label tag so coverage reads as part of the same measurement report as the
    SonarCloud summary. The user can run `python3 coverage_summary.py lcov.info
    --verbose` for the full list.
    """
    verbose = False
    label = 'Local Coverage'
    positional = []
    args = list(sys.argv[1:])
    i = 0
    while i < len(args):
        arg = args[i]
        if arg in ('--verbose', '--all', '-v'):
            verbose = True
            i += 1
        elif arg in ('-h', '--help'):
            print(__doc__)
            sys.exit(0)
        elif arg == '--label' and i + 1 < len(args):
            label = args[i + 1]
            i += 2
        else:
            positional.append(arg)
            i += 1

    if not positional:
        print("Usage: coverage_summary.py <lcov.info> [repo_root] [--verbose] [--label NAME]")
        sys.exit(1)

    coverage_path = positional[0]
    repo_root = positional[1] if len(positional) > 1 else os.getcwd()

    if not os.path.isfile(coverage_path):
        print('  No coverage yet. Run: make sonar-scan')
        sys.exit(0)

    try:
        display(coverage_path, repo_root, verbose=verbose, label=label)
    except (OSError, ValueError) as exc:
        print('  Failed to read coverage ({}): {}'.format(coverage_path, exc), file=sys.stderr)
        print('  Re-run with: make coverage-run')
        sys.exit(0)


if __name__ == '__main__':
    main()
