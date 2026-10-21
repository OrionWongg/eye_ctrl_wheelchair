#!/usr/bin/env python3
"""Create a stationary-face video from MediaPipe's public portrait test asset."""
from pathlib import Path
import hashlib
import json
import cv2

base = Path(__file__).resolve().parents[1] / 'test' / 'fixtures'
image = cv2.imread(str(base / 'portrait.jpg'))
if image is None:
    raise SystemExit('Missing test/fixtures/portrait.jpg; see OFFLINE.md for its official source.')
scale = 640.0 / image.shape[1]
image = cv2.resize(image, (640, int(image.shape[0]*scale)//2*2))
video = cv2.VideoWriter(str(base/'portrait.avi'), cv2.VideoWriter_fourcc(*'MJPG'), 20.0, (image.shape[1], image.shape[0]))
if not video.isOpened(): raise RuntimeError('MJPG VideoWriter unavailable')
for _ in range(140): video.write(image)
video.release()
meta = {'source': 'https://storage.googleapis.com/mediapipe-assets/portrait.jpg',
        'sha256': hashlib.sha256((base/'portrait.jpg').read_bytes()).hexdigest(),
        'video': 'portrait.avi', 'frames': 140, 'fps': 20,
        'description': 'Stationary real face: verifies inference and ROS transport, not deliberate blinking.'}
(base/'provenance.json').write_text(json.dumps(meta, indent=2)+'\n')
print(meta)
