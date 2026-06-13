#!/usr/bin/env python3
"""Convert lcov to SonarQube generic coverage XML format."""
import re
import sys


def convert(lcov_path, xml_path):
    with open(lcov_path) as f:
        lcov = f.read()

    # Parse lcov
    files = {}
    current_file = None
    for line in lcov.split('\n'):
        if line.startswith('SF:'):
            current_file = line[3:]
            # Strip absolute path prefix to make relative
            m = re.search(r'engine-sim-bridge/(.*)', current_file)
            if m:
                current_file = m.group(1)
            files[current_file] = {}
        elif current_file and line.startswith('DA:'):
            parts = line[3:].split(',')
            if len(parts) >= 2:
                try:
                    line_no = int(parts[0])
                    count = int(parts[1])
                    files[current_file][line_no] = count
                except ValueError:
                    pass

    # Generate SonarQube generic coverage XML (only src/ files)
    xml_lines = ['<?xml version="1.0" encoding="UTF-8"?>', '<coverage version="1">']
    for filepath in sorted(files.keys()):
        if not filepath.startswith('src/'):
            continue
        lines = files[filepath]
        if not lines:
            continue
        xml_lines.append(f'  <file path="{filepath}">')
        for line_no in sorted(lines.keys()):
            covered = "true" if lines[line_no] > 0 else "false"
            xml_lines.append(f'    <lineToCover lineNumber="{line_no}" covered="{covered}"/>')
        xml_lines.append('  </file>')
    xml_lines.append('</coverage>')
    xml_lines.append('')

    with open(xml_path, 'w') as f:
        f.write('\n'.join(xml_lines))

    # Stats
    total = sum(len(d) for f, d in files.items() if f.startswith('src/'))
    covered = sum(1 for f, d in files.items() if f.startswith('src/') for c in d.values() if c > 0)
    file_count = len([f for f in files if f.startswith('src/')])
    print(f"Generated {xml_path}: {file_count} files, {covered}/{total} covered lines ({covered/total*100:.1f}%)" if total else "No coverage data")


if __name__ == '__main__':
    if len(sys.argv) < 3:
        print("Usage: lcov_to_xml.py <input.lcov> <output.xml>")
        sys.exit(1)
    convert(sys.argv[1], sys.argv[2])
