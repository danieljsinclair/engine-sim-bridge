#!/usr/bin/env python3
"""Display local coverage summary from lcov.info.

Parses the lcov produced by run_coverage_tests.sh and prints:
  - overall coverage (all SF records)
  - src-only coverage (SF records under /src/)
  - per-file coverage for src files, sorted lowest-first

This is the LOCAL ground truth — it reflects exactly what llvm-cov measured
in build-cov. It is independent of SonarCloud's dashboard number, which also
counts dead-stripped src files (absent from lcov) as fully uncovered.

SINGLE SOURCE OF TRUTH: the "src/ only (sonar scope)" line applies the SAME
exclusions SonarCloud uses. It reads sonar-project.properties from repo_root,
parses sonar.exclusions + sonar.coverage.exclusions (Ant-style globs, comma
separated), and drops any SF whose repo-root-relative path matches. That way
the local number matches what SonarCloud counts (sonar.sources=src minus the
exclusion globs), instead of counting every /src/ SF in the lcov.

Note: a src file that no build-cov test binary references has no coverage
mapping, so it is ABSENT from lcov entirely (not "0%"). Such files show up
on SonarCloud as 0% because Sonar counts them against sonar.sources=src.
They are listed below under "src files absent from lcov" when they can be
discovered on disk; those are the candidates whose coverage gap is real.
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


def display(coverage_path, repo_root):
    """Print the coverage summary."""
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
    sonar_hit, sonar_found, sonar_pct = summarize(sonar_records)

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
            sonar_pct, RESET, GREY + '(matches SonarCloud)' + RESET))
        print('  {}  exclusions:{} {}{}'.format(
            GREY, RESET, props_path, ''))
    else:
        print('  {}src/ sonar scope{:11} {}no sonar-project.properties — same as src/ only{}'.format(
            BOLD, RESET, GREY, RESET))
    print('')

    # Per-file src coverage, lowest-first. Cap output to keep it readable.
    # Use the sonar scope so the table reflects what SonarCloud counts.
    src_file_records = sorted(sonar_records, key=lambda r: (r[2] / r[1] if r[1] else 0.0))
    print('  src/ per-file (sonar scope, lowest coverage first):')
    for path, lf, lh in src_file_records:
        rel = os.path.relpath(path, repo_root) if repo_root else path
        pct = (100.0 * lh / lf) if lf else 0.0
        print('    {}{:5.1f}%{:8} {}/{}  {}{}'.format(
            coverage_color(pct), pct, RESET, lh, lf, rel, ''))
    print('')

    # Dead-stripped src files: on disk but absent from lcov. These are the
    # real 0% files on SonarCloud (genuine coverage gaps, not a measurement bug).
    absent, scanned = find_src_files_on_disk(
        [r[0] for r in sonar_records], repo_root)
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

    print('  ' + BOLD + 'Note:' + RESET + ' the sonar-scope line is the single source of truth;')
    print('  it applies the same exclusions as SonarCloud.')
    print('')


def main():
    if len(sys.argv) < 2:
        print("Usage: coverage_summary.py <lcov.info> [repo_root]")
        sys.exit(1)

    coverage_path = sys.argv[1]
    repo_root = sys.argv[2] if len(sys.argv) > 2 else os.getcwd()

    if not os.path.isfile(coverage_path):
        print('  No coverage yet. Run: make sonar-scan')
        sys.exit(0)

    try:
        display(coverage_path, repo_root)
    except (OSError, ValueError) as exc:
        print('  Failed to read coverage ({}): {}'.format(coverage_path, exc), file=sys.stderr)
        print('  Re-run with: make coverage-run')
        sys.exit(0)


if __name__ == '__main__':
    main()
