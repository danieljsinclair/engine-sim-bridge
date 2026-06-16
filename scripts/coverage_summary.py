#!/usr/bin/env python3
"""Display local coverage summary from lcov.info.

Parses the lcov produced by run_coverage_tests.sh and prints:
  - overall coverage (all SF records)
  - src-only coverage (SF records under /src/)
  - src-only sonar-scope coverage (matches SonarCloud)

OUTPUT MODES:
  Default (concise, used by `make`): the three headline numbers + the TOP 5
  worst (lowest coverage) src files + the MISSING (dead-stripped) files list.
  --verbose / --all : prints the full per-file table for every src file.

SONAR-SCOPE DENOMINATOR: the "src/ sonar scope" line applies the SAME
exclusions SonarCloud uses (sonar.exclusions + sonar.coverage.exclusions from
sonar-project.properties) AND includes dead-stripped files at 0%. A dead-
stripped file is one that is compiled but not linked into any test binary
(no caller), so it has no coverage mapping and is ABSENT from lcov. SonarCloud
counts these against sonar.sources=src at 0%, so they are added to the local
denominator (LH=0) so the local number tracks SonarCloud. Their line count is
approximated (no coverage mapping exists for an exact LF).

A small residual gap vs SonarCloud is normal: Sonar and llvm-cov count
executable lines slightly differently per file (lcov drift), independent of
dead-stripping.
"""
import fnmatch
import os
import re
import sys

RED = '\033[31m'
ORANGE = '\033[38;5;208m'
YELLOW = '\033[33m'
GREEN = '\033[32m'
CYAN = '\033[36m'
GREY = '\033[90m'
BOLD = '\033[1m'
RESET = '\033[0m'


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
    """Approximate executable line count for a source file with no lcov data.

    Counts non-blank, non-comment lines (stripping // and /* */ comments and
    standalone braces/symbols). This is an APPROXIMATION of what llvm-cov's LF
    would record — it cannot be exact without coverage mapping, but it lets the
    sonar-scope denominator include dead-stripped files at 0% so the local
    number tracks SonarCloud (which counts them as uncovered). Returns 0 on
    read failure (file treated as no contribution).
    """
    count = 0
    in_block = False
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
                # Skip bare structural tokens llvm-cov wouldn't count.
                if line in ('{', '}', '};', '};'):
                    continue
                count += 1
    except OSError:
        return 0
    return count


def display(coverage_path, repo_root, verbose=False):
    """Print the coverage summary.

    By default prints a CONCISE view: the three headline numbers (overall,
    src/ only, src/ sonar scope), the TOP 5 worst (lowest coverage) src files,
    and the MISSING (dead-stripped) files list. With verbose=True prints the
    full per-file table instead of just the top 5.
    """
    records = parse_lcov(coverage_path)

    all_hit, all_found, all_pct = summarize(records)
    src_records = [r for r in records if is_src(r[0])]
    src_hit, src_found, src_pct = summarize(src_records)

    # Sonar scope: src/** minus sonar.exclusions + sonar.coverage.exclusions.
    # This is the single source of truth — it matches what SonarCloud counts.
    patterns, props_path = load_sonar_exclusions(repo_root)
    if patterns:
        sonar_records = [r for r in src_records
                         if not matches_exclusions(r[0], repo_root, patterns)]
    else:
        sonar_records = src_records

    # Dead-stripped src files: on disk but absent from lcov. SonarCloud counts
    # these against sonar.sources=src at 0%, so they MUST be in the denominator
    # for the local number to track SonarCloud. We approximate their LF (no
    # coverage mapping exists to give an exact count) and add them at LH=0.
    absent, scanned = find_src_files_on_disk(
        [r[0] for r in sonar_records], repo_root)
    dead_added_lines = 0
    dead_records = []
    dead_paths = set()
    if scanned and absent:
        for path in absent:
            approx_lf = count_code_lines(path)
            if approx_lf > 0:
                dead_records.append((path, approx_lf, 0))
                dead_added_lines += approx_lf
                dead_paths.add(path)

    sonar_with_dead = list(sonar_records) + dead_records
    sonar_hit, sonar_found, sonar_pct = summarize(sonar_with_dead)

    print('')
    print('=== Local Coverage Summary ===')
    print('  source: {}'.format(coverage_path))
    print('')
    print('  {}Overall{:18} {}{}/{} lines ({:.1f}%){}'.format(
        BOLD, RESET, coverage_color(all_pct), all_hit, all_found, all_pct, RESET))
    print('  {}src/ only{:17} {}{}/{} lines ({:.1f}%){}'.format(
        BOLD, RESET, coverage_color(src_pct), src_hit, src_found, src_pct, RESET))
    if patterns:
        print('  {}src/ sonar scope{:11} {}{}/{} lines ({:.1f}%){} {}'.format(
            BOLD, RESET, coverage_color(sonar_pct), sonar_hit, sonar_found,
            sonar_pct, RESET, GREY + '(tracks SonarCloud)' + RESET))
        if dead_added_lines:
            print('  {}  incl. dead-stripped:{} {} files, {} lines at 0% ({} approx){}'.format(
                GREY, RESET, len(dead_records), dead_added_lines,
                'line count', ''))
        print('  {}  exclusions:{} {}{}'.format(
            GREY, RESET, props_path, ''))
    else:
        print('  {}src/ sonar scope{:11} {}no sonar-project.properties — same as src/ only{}'.format(
            BOLD, RESET, GREY, RESET))
    print('')

    # Per-file src coverage, lowest-first. CONCISE by default (top 5 worst);
    # --verbose shows the full table. Uses the sonar scope (includes dead files
    # at 0% so the ranking reflects genuine coverage gaps).
    src_file_records = sorted(
        sonar_with_dead, key=lambda r: (r[2] / r[1] if r[1] else 0.0))
    if verbose:
        print('  src/ per-file (sonar scope, lowest coverage first):')
    else:
        print('  src/ top 5 worst (sonar scope): {}--verbose for full list{}'.format(
            GREY, RESET))
    shown = src_file_records if verbose else src_file_records[:5]
    for path, lf, lh in shown:
        rel = os.path.relpath(path, repo_root) if repo_root else path
        pct = (100.0 * lh / lf) if lf else 0.0
        marker = '  {}DEAD{} '.format(RED, RESET) if path in dead_paths else ''
        print('    {}{:5.1f}%{:8} {}/{}  {}{}{}'.format(
            coverage_color(pct), pct, RESET, lh, lf, marker, rel, ''))
    print('')

    # Dead-stripped src files: on disk but absent from lcov. These are the
    # real 0% files on SonarCloud (genuine coverage gaps, not a measurement bug).
    if scanned:
        print('  src files absent from lcov (dead-stripped — 0% on SonarCloud):')
        if absent:
            for path in absent:
                rel = os.path.relpath(path, repo_root) if repo_root else path
                print('    {}MISSING{}  {}'.format(RED, RESET, rel))
        else:
            print('    {}none — every src/*.cpp has coverage data{}'.format(
                GREEN, RESET))
        print('')

    print('  ' + BOLD + 'Note:' + RESET +
          ' dead-stripped = compiled but not linked into any test binary')
    print('  (no caller) -> absent from lcov, counted as 0% by SonarCloud. The')
    print('  sonar-scope line includes them so local tracks SonarCloud. A small')
    print('  residual gap is normal: Sonar and llvm-cov count executable lines')
    print('  slightly differently per file.')
    print('')


def main():
    """Entry point. Accepts an optional --verbose/--all flag anywhere in argv.

    Usage: coverage_summary.py <lcov.info> [repo_root] [--verbose|--all]
    The Makefile calls this with just the lcov path (concise output). The user
    can run `python3 coverage_summary.py lcov.info --verbose` for the full list.
    """
    verbose = False
    positional = []
    for arg in sys.argv[1:]:
        if arg in ('--verbose', '--all', '-v'):
            verbose = True
        elif arg in ('-h', '--help'):
            print(__doc__)
            sys.exit(0)
        else:
            positional.append(arg)

    if not positional:
        print("Usage: coverage_summary.py <lcov.info> [repo_root] [--verbose]")
        sys.exit(1)

    coverage_path = positional[0]
    repo_root = positional[1] if len(positional) > 1 else os.getcwd()

    if not os.path.isfile(coverage_path):
        print('  No coverage yet. Run: make sonar-scan')
        sys.exit(0)

    try:
        display(coverage_path, repo_root, verbose=verbose)
    except (OSError, ValueError) as exc:
        print('  Failed to read coverage ({}): {}'.format(coverage_path, exc), file=sys.stderr)
        print('  Re-run with: make coverage-run')
        sys.exit(0)


if __name__ == '__main__':
    main()
