#!/usr/bin/env python3
"""Display SonarCloud issue summary from cached report."""
import json, sys

RED = '\033[31m'
ORANGE = '\033[38;5;208m'
YELLOW = '\033[33m'
GREEN = '\033[32m'
CYAN = '\033[36m'
BOLD = '\033[1m'
RESET = '\033[0m'


def severity_color(sev):
    """Return ANSI color code for a severity level."""
    return total_issues_color({sev: 1})


def total_issues_color(counts):
    """Return ANSI color code for total issues based on severity breakdown."""
    if counts.get('CRITICAL', 0) > 0:
        return RED
    elif counts.get('MAJOR', 0) > 0:
        return ORANGE
    elif counts.get('MINOR', 0) > 0:
        return YELLOW
    elif counts.get('INFO', 0) > 0:
        return CYAN

    return GREEN


def format_severity_line(sev, count):
    """Print a single severity line with color."""
    print('    {}● {:<8}: {}{}'.format(severity_color(sev), sev, count, RESET))


def format_issue_line(issue):
    """Print a single issue line with color."""
    sev = issue.get('severity', '?')
    msg = issue.get('message', '')[:60]
    comp = issue.get('component', '').split(':')[-1]
    line = issue.get('line', '')
    loc = comp + (':' + str(line) if line else '')
    print('    {}[{}]{} {} - {}'.format(severity_color(sev), sev, RESET, loc, msg))


def count_by_severity(issues):
    """Return dict of severity -> count."""
    counts = {}
    for sev in ['CRITICAL', 'MAJOR', 'MINOR', 'INFO']:
        counts[sev] = sum(1 for i in issues if i.get('severity') == sev)
    return counts


def display_summary(data):
    """Display the full SonarCloud issue summary."""
    issues = data.get('issues', [])
    total = data.get('total', len(issues))
    counts = count_by_severity(issues)

    print('')
    print('=== SonarCloud Summary ===')

    if total > 0:
        print('  {}Total issues: {}{}'.format(total_issues_color(counts), total, RESET))
        print('')
        print('  Issues by severity:')
        for sev in ['CRITICAL', 'MAJOR', 'MINOR', 'INFO']:
            format_severity_line(sev, counts[sev])
        print('')
        print('  Top issues:')
        for issue in issues[:10]:
            format_issue_line(issue)
    else:
        print('  {}No issues found.{}'.format(GREEN, RESET))

    print('')


def load_report(path):
    """Load and validate the SonarCloud report JSON."""
    try:
        with open(path) as f:
            return json.load(f)
    except (FileNotFoundError, json.JSONDecodeError):
        return None


def main():
    if len(sys.argv) < 2:
        print("Usage: sonar_summary.py <report.json>")
        sys.exit(1)

    data = load_report(sys.argv[1])
    if data is None:
        print("  No report yet. Run: make sonar-scan")
        sys.exit(0)

    display_summary(data)


if __name__ == '__main__':
    main()
