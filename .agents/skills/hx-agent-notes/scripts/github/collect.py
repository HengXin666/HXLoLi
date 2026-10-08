#!/usr/bin/env python3
"""Collect strict scan diagnostics without failing the GitHub job."""
import argparse
import json
import os
import subprocess
from pathlib import Path


def collect(skill, output, head, flags):
    gate = skill / 'scripts/redline'
    command = ['uv', 'run', '--with-requirements', str(gate / 'requirements.txt'), 'python',
               str(gate / 'verify.py'), *flags, '--head', head, '--json', str(output)]
    try:
        output.unlink(missing_ok=True)
        if '--base' in flags and not flags[flags.index('--base') + 1]:
            raise ValueError('Comparison base unavailable; refusing an empty diff')
        process = subprocess.run(command, timeout=600)
        data = json.loads(output.read_text())
        if (not isinstance(data, dict) or data.get('version') != 2 or data.get('head') != head
                or not isinstance(data.get('issues'), list) or not isinstance(data.get('ok'), bool)
                or data['ok'] != (not data['issues'])):
            raise ValueError('Scanner did not produce a matching diagnostic report')
        if process.returncode not in (0, 1, 2) or (process.returncode != 0 and not data['issues']):
            raise ValueError(f'Scanner exited {process.returncode} without valid diagnostics')
    except (OSError, ValueError, subprocess.TimeoutExpired) as exc:
        data = dict(version=2, ok=False, head=head, issues=[dict(rule='scanner-error',
                    path='.agents/notes.config.json', line=1, severity='error', related=[], message=str(exc))])
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(data, ensure_ascii=False, indent=2) + '\n')
    summary = os.environ.get('GITHUB_STEP_SUMMARY')
    if summary:
        with open(summary, 'a') as stream:
            stream.write(f"Agent Notes: {len(data['issues'])} findings. See the diagnostic artifact and code comments.\n")
    return data


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--skill', type=Path, required=True)
    parser.add_argument('--output', type=Path, default=Path('agent-notes-report.json'))
    parser.add_argument('--head', required=True)
    args, flags = parser.parse_known_args()
    collect(args.skill, args.output, args.head, flags)


if __name__ == '__main__':
    main()
