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
    sev = issue.get('severity') or '?'
    msg = (issue.get('message') or '')[:60]
    comp = (issue.get('component') or '').split(':')[-1]
    line = issue.get('line')
    loc = comp + (':' + str(line) if line else '')
    print('    {}[{}]{} {} - {}'.format(severity_color(sev), sev, RESET, loc, msg))


def count_by_severity(issues):
    """Return dict of severity -> count."""
    counts = {}
    for sev in ['CRITICAL', 'MAJOR', 'MINOR', 'INFO']:
        counts[sev] = sum(1 for i in issues if i.get('severity') == sev)
    return counts


def count_by_impact(issues):
    """Return dict of software quality -> impact severity -> count."""
    counts = {}
    for i in issues:
        for impact in i.get('impacts', []):
            quality = impact.get('softwareQuality') or 'UNKNOWN'
            severity = impact.get('severity') or 'UNKNOWN'
            counts.setdefault(quality, {})[severity] = counts.setdefault(quality, {}).get(severity, 0) + 1
    return counts


def count_impact_severity(issues, quality):
    """Return dict of impact severity -> count for a software quality."""
    counts = {}
    for i in issues:
        for impact in i.get('impacts', []):
            if impact.get('softwareQuality') != quality:
                continue
            severity = impact.get('severity') or 'UNKNOWN'
            counts[severity] = counts.get(severity, 0) + 1
    return counts


def impact_severity_color(severity):
    """Return ANSI color code for impact severity."""
    if severity == 'BLOCKER':
        return RED
    if severity == 'HIGH':
        return RED
    if severity == 'MEDIUM':
        return ORANGE
    if severity == 'LOW':
        return YELLOW
    if severity == 'INFO':
        return CYAN
    return GREEN


def format_impact_line(quality, counts):
    """Print a software-quality impact line with color."""
    total = sum(counts.values())
    print('  {}● {:<13}: {}{}'.format(total_issues_color(counts), quality, total, RESET))


def display_summary(data):
    """Display the full SonarCloud issue summary."""
    issues = [i for i in data.get('issues', []) if i.get('status') not in ('CLOSED', 'RESOLVED')]
    total = len(issues)
    counts = count_by_severity(issues)
    impacts = count_by_impact(issues)

    print('')
    print('=== SonarCloud Summary ===')

    if total > 0:
        print('  {}Total issues: {}{}'.format(total_issues_color(counts), total, RESET))
        print('')
        print('  Issues by severity:')
        for sev in ['CRITICAL', 'MAJOR', 'MINOR', 'INFO']:
            format_severity_line(sev, counts[sev])
        print('')
        print('  Issues by software quality:')
        for quality in ['RELIABILITY', 'MAINTAINABILITY', 'SECURITY', 'UNKNOWN']:
            if quality in impacts:
                format_impact_line(quality, impacts[quality])
                severity_counts = count_impact_severity(issues, quality)
                for severity in ['BLOCKER', 'HIGH', 'MEDIUM', 'LOW', 'INFO', 'UNKNOWN']:
                    if severity in severity_counts:
                        print('      {}● {:<8}: {}{}'.format(impact_severity_color(severity), severity, severity_counts[severity], RESET))
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
