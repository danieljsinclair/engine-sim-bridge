#!/usr/bin/env python3
"""Display local coverage summary from lcov.info.

Parses the lcov produced by run_coverage_tests.sh and prints:
  - overall coverage (all SF records)
  - src-only coverage (SF records under /src/)
  - per-file coverage for src files, sorted lowest-first

This is the LOCAL ground truth — it reflects exactly what llvm-cov measured
in build-cov. It is independent of SonarCloud's dashboard number, which also
counts dead-stripped src files (absent from lcov) as fully uncovered.

Note: a src file that no build-cov test binary references has no coverage
mapping, so it is ABSENT from lcov entirely (not "0%"). Such files show up
on SonarCloud as 0% because Sonar counts them against sonar.sources=src.
They are listed below under "src files absent from lcov" when they can be
discovered on disk; those are the candidates whose coverage gap is real.
"""
import os
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

    print('')
    print('=== Local Coverage Summary ===')
    print('  source: {}'.format(coverage_path))
    print('')
    print('  {}Overall{:18} {}{}/{} lines ({:.1f}%){}'.format(
        BOLD, RESET, coverage_color(all_pct), all_hit, all_found, all_pct, RESET))
    print('  {}src/ only{:17} {}{}/{} lines ({:.1f}%){}'.format(
        BOLD, RESET, coverage_color(src_pct), src_hit, src_found, src_pct, RESET))
    print('')

    # Per-file src coverage, lowest-first. Cap output to keep it readable.
    src_file_records = sorted(src_records, key=lambda r: (r[2] / r[1] if r[1] else 0.0))
    print('  src/ per-file (lowest coverage first):')
    for path, lf, lh in src_file_records:
        rel = os.path.relpath(path, repo_root) if repo_root else path
        pct = (100.0 * lh / lf) if lf else 0.0
        print('    {}{:5.1f}%{:8} {}/{}  {}{}'.format(
            coverage_color(pct), pct, RESET, lh, lf, rel, ''))
    print('')

    # Dead-stripped src files: on disk but absent from lcov. These are the
    # real 0% files on SonarCloud (genuine coverage gaps, not a measurement bug).
    absent, scanned = find_src_files_on_disk(
        [r[0] for r in src_records], repo_root)
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

    print('  ' + BOLD + 'Note:' + RESET + ' src % above excludes dead-stripped files;')
    print('  SonarCloud counts them as uncovered, so its overall % is lower.')
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
