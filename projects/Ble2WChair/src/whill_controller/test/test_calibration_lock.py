"""Regression test for the visually validated WHILL center calibration."""

import json
from pathlib import Path


def test_locked_center_sequence():
    """Keep every APP initialization on the approved deterministic target."""
    config_path = (
        Path(__file__).parents[1] / 'config' / 'default_calibration.json')
    with config_path.open(encoding='utf-8') as config_file:
        dataset = json.load(config_file)

    center = next(
        point for point in dataset['points']
        if point['name'] == 'steering wheel center')

    assert dataset['origin'] == 'bottom-right screen boundary'
    assert center['sequence'] == ('a' * 25) + ('w' * 14)
    assert center['counts'] == {'a': 25, 'w': 14}
    assert center['total_count'] == 39


def test_direction_offsets():
    """Keep the eight APP targets on their approved center-relative offsets."""
    config_path = (
        Path(__file__).parents[1] / 'config' / 'default_calibration.json')
    with config_path.open(encoding='utf-8') as config_file:
        dataset = json.load(config_file)

    offsets = {
        point['name']: point['offset']
        for point in dataset['points']
    }
    assert offsets == {
        'steering wheel center': {'x': 0, 'y': 0},
        'forward': {'x': 1, 'y': -6},
        'backward': {'x': 0, 'y': 6},
        'left': {'x': -5, 'y': 0},
        'right': {'x': 6, 'y': 0},
        'left-forward': {'x': -4, 'y': -5},
        'left-backward': {'x': -4, 'y': 4},
        'right-forward': {'x': 4, 'y': -4},
        'right-backward': {'x': 5, 'y': 4},
    }

    sequences = {
        point['name']: point['sequence']
        for point in dataset['points']
        if point['name'] != 'steering wheel center'
    }
    assert sequences == {
        'forward': 'wwwwwwd',
        'backward': 'ssssss',
        'left': 'aaaaa',
        'right': 'dddddd',
        'left-forward': 'wwwwwaaaa',
        'left-backward': 'aaaassss',
        'right-forward': 'wwwwdddd',
        'right-backward': 'dddddssss',
    }
