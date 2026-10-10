import numpy as np

from e1r_person_detection.detector import DetectorParams, PersonDetector
from e1r_person_detection.synthetic import render
from e1r_person_detection.tracker import Tracker


def _learned_detector(rng, **kw):
    det = PersonDetector(DetectorParams(**kw))
    for _ in range(det.p.background_learn_frames):
        det.process(render([], rng=rng)[0])
    return det


def test_ground_plane_recovers_mounting():
    rng = np.random.default_rng(0)
    det = PersonDetector(DetectorParams(use_background=False))
    det.process(render([], height=1.2, pitch_deg=10.0, rng=rng)[0])
    n, d = det.plane
    assert abs(d - 1.2) < 0.05
    assert abs(np.degrees(np.arccos(n[2])) - 10.0) < 1.5


def test_empty_scene_has_no_person():
    rng = np.random.default_rng(1)
    det = _learned_detector(rng)
    for _ in range(5):
        res = det.process(render([], rng=rng)[0])
        assert not any(d.is_person for d in res.detections)


def test_static_objects_rejected_by_shape_without_background():
    rng = np.random.default_rng(2)
    det = PersonDetector(DetectorParams(use_background=False))
    res = det.process(render([], rng=rng)[0])
    assert not any(d.is_person for d in res.detections)


def test_people_detected_at_various_ranges():
    rng = np.random.default_rng(3)
    det = _learned_detector(rng)
    for x, y in [(2.0, 0.0), (5.0, 1.5), (7.5, -2.0)]:
        res = det.process(render([(x, y, 0.0, 1.75)], rng=rng)[0])
        persons = [d for d in res.detections if d.is_person]
        assert len(persons) == 1, (x, y, [d.reason for d in res.detections])
        assert np.linalg.norm(persons[0].position_ground - [x, y]) < 0.3
        assert abs(persons[0].size[2] - 1.75) < 0.15


def test_two_people_separated():
    rng = np.random.default_rng(4)
    det = _learned_detector(rng)
    res = det.process(render([(4.0, -1.0, 0.0, 1.8), (4.5, 1.0, 1.0, 1.6)], rng=rng)[0])
    assert sum(d.is_person for d in res.detections) == 2


def test_tracker_confirms_and_keeps_id():
    rng = np.random.default_rng(5)
    det = _learned_detector(rng)
    tr = Tracker(confirm_hits=3)
    ids = []
    for k in range(10):
        res = det.process(render([(5.0, -2.0 + 0.1 * k, k, 1.75)], rng=rng)[0])
        out = tr.update([d for d in res.detections if d.is_person], t=k * 0.1)
        if k < 2:
            assert out == []
        else:
            assert len(out) == 1
            ids.append(out[0].id)
    assert len(set(ids)) == 1
