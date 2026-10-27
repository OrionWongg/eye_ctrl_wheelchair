"""Resolve known eye-camera hub layouts without relying on video numbering."""
from pathlib import Path


def fusion_camera_defaults(paths=None):
    paths = sorted(str(p) for p in (
        Path('/dev/v4l/by-path').glob('*-video-index0') if paths is None else paths))
    layouts = []
    for hub in ('4.4', '4.1'):
        pair = {}
        for side, port in (('left', 1), ('right', 2)):
            matches = [p for p in paths if f':{hub}.{port}:1.0-video-index0' in p]
            if len(matches) == 1:
                pair[side] = matches[0]
        if pair:
            layouts.append(pair)
    # Multiple populated hubs cannot establish a unique left/right assignment.
    return layouts[0] if len(layouts) == 1 else {}
