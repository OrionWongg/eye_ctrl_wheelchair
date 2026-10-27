"""USB hub changes must not depend on unstable video device numbering."""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parents[1] / 'tools'))
from camera_paths import fusion_camera_defaults


def device(port):
    return f'/dev/v4l/by-path/platform-3610000.usb-usb-0:{port}:1.0-video-index0'


def test_original_and_current_hubs():
    for hub in ('4.4', '4.1'):
        left, right = device(hub + '.1'), device(hub + '.2')
        assert fusion_camera_defaults([right, left]) == dict(left=left, right=right)


def test_no_guess_for_unknown_or_ambiguous_hubs():
    assert fusion_camera_defaults([device('8.1')]) == {}
    assert fusion_camera_defaults([device('4.4.1'), device('4.1.1')]) == {}


def test_single_eye_and_metadata_node():
    left = device('4.1.1')
    assert fusion_camera_defaults([left, left.replace('index0', 'index1')]) == dict(left=left)
