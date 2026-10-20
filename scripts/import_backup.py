#!/usr/bin/env python3
"""Import a backup without overwriting files, and record unreadable/damaged files."""
import argparse
import ast
import hashlib
import json
import os
from pathlib import Path
import shutil

SKIP = {'.git', 'build', 'install', 'log', '__pycache__', '.pytest_cache',
        'artifacts', 'model-results'}
TEXT = {'.py', '.sh', '.bash', '.xml', '.md', '.txt', '.json', '.yaml',
        '.yml', '.cfg', '.sdf', '.urdf', '.cpp', '.h', '.hpp', '.cmake'}


def import_backup(source, destination):
    records = []

    def error(exc):
        records.append({'path': os.path.relpath(exc.filename, source),
                        'status': 'unreadable', 'error': str(exc)})

    for directory, dirs, files in os.walk(source, onerror=error):
        relative = Path(directory).relative_to(source)
        dirs[:] = sorted(d for d in dirs if d not in SKIP and not d.startswith('.venv')
                         and not (relative.name == 'vendor' and d in {'runtime', 'smac'}))
        for name in sorted(files):
            path = Path(directory) / name
            rel = path.relative_to(source)
            item = {'path': str(rel)}
            try:
                if path.is_symlink():
                    item.update(status='symlink_skipped', target=os.readlink(path))
                else:
                    data = path.read_bytes()
                    item.update(size=len(data), sha256=hashlib.sha256(data).hexdigest())
                    if data and not data.strip(b'\0'):
                        item['status'] = 'zero_filled'
                    elif path.suffix in TEXT and b'\0' in data:
                        item['status'] = 'null_bytes_in_text'
                    else:
                        if path.suffix == '.py':
                            ast.parse(data, filename=str(rel))
                        target = destination / rel
                        if target.exists():
                            item['status'] = ('already_present' if target.read_bytes() == data
                                              else 'conflict_not_overwritten')
                        else:
                            target.parent.mkdir(parents=True, exist_ok=True)
                            shutil.copy2(path, target)
                            item['status'] = 'copied'
            except (OSError, SyntaxError, ValueError) as exc:
                item.update(status='unreadable_or_invalid', error=str(exc))
            records.append(item)
    return records


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('--destination', type=Path, default=Path(__file__).resolve().parents[1])
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    source, destination = args.source.resolve(), args.destination.resolve()
    if not source.is_dir() or source == destination or source in destination.parents or destination in source.parents:
        parser.error('Source must exist and source/destination must not contain each other')
    records = import_backup(source, destination)
    args.report.parent.mkdir(parents=True, exist_ok=True)
    args.report.write_text(json.dumps(records, ensure_ascii=False, indent=2) + '\n')
    from collections import Counter
    print(json.dumps(dict(Counter(r['status'] for r in records)), ensure_ascii=False))


if __name__ == '__main__':
    main()
