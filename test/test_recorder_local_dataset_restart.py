"""Local recording must survive an exit before the first episode is saved."""
import json

import numpy as np
import pytest

from data_collection import lerobot_collection as recorder


def packet():
    return {
        "robot_state_dim": 7, "action_dim": 7, "arm_mode": "left",
        "include_right_arm": False, "include_gripper": False, "include_hand": False,
        "arm_action_representation": "delta_joint_position",
        "camera_names": ["cam_left"],
        "cameras": {"cam_left": {"shape": (16, 16, 3),
                                  "rgb": np.zeros((16, 16, 3), dtype=np.uint8)}},
    }


def create(root):
    return recorder.make_dataset(packet(), "local/restart-test", 15, root, "h264")


def test_quit_before_first_save_does_not_load_empty_dataset_stats(monkeypatch, tmp_path):
    dataset, _, _ = create(tmp_path / "dataset")

    def unexpected_stats(*args, **kwargs):
        pytest.fail("there are no saved episodes to compute statistics for")

    monkeypatch.setattr(recorder, "ensure_dataset_stats", unexpected_stats)
    recorder.finalize_dataset(dataset, "local/restart-test")
    assert dataset.meta.total_episodes == 0


def test_empty_dataset_restart_preserves_original_and_never_resumes(monkeypatch, tmp_path):
    root = tmp_path / "dataset"
    first, _, resumed = create(root)
    first.finalize()
    assert not resumed
    original = {p.relative_to(root): p.read_bytes() for p in root.rglob("*") if p.is_file()}
    # Even potentially recoverable files in a zero-episode dataset must survive.
    (root / "uncommitted.txt").write_text("keep me")
    (tmp_path / "dataset_restart").mkdir()

    def unexpected_resume(**kwargs):
        pytest.fail("empty local dataset must not enter the Hub fallback")

    monkeypatch.setattr(recorder.LeRobotDataset, "resume", unexpected_resume)
    replacement, cameras, resumed = create(root)
    try:
        assert replacement.root == tmp_path / "dataset_restart_1"
        assert cameras == ["cam_left"] and not resumed
        assert (root / "uncommitted.txt").read_text() == "keep me"
        assert all((root / name).read_bytes() == contents for name, contents in original.items())
        assert recorder.is_lerobot_dataset_root(replacement.root)
    finally:
        replacement.finalize()


@pytest.mark.parametrize("missing", ["tasks", "episodes"])
def test_saved_dataset_missing_metadata_fails_locally_without_resuming(monkeypatch, tmp_path, missing):
    root = tmp_path / "dataset"
    dataset, _, _ = create(root)
    dataset.finalize()
    info_path = root / recorder.LEROBOT_INFO_PATH
    info = json.loads(info_path.read_text())
    info.update(total_episodes=1, total_frames=3)
    info_path.write_text(json.dumps(info))
    if missing != "tasks":
        (root / "meta/tasks.parquet").touch()
    if missing != "episodes":
        episodes = root / "meta/episodes/chunk-000"
        episodes.mkdir(parents=True)
        (episodes / "file-000.parquet").touch()

    def unexpected_resume(**kwargs):
        pytest.fail("incomplete saved data must not enter the Hub fallback")

    monkeypatch.setattr(recorder.LeRobotDataset, "resume", unexpected_resume)
    with pytest.raises(FileNotFoundError, match="no Hub download"):
        create(root)
    assert json.loads(info_path.read_text())["total_episodes"] == 1
    assert not (tmp_path / "dataset_restart").exists()


def test_saved_local_dataset_still_uses_resume(monkeypatch, tmp_path):
    root = tmp_path / "dataset"
    dataset, _, _ = create(root)
    dataset.finalize()
    info_path = root / recorder.LEROBOT_INFO_PATH
    info = json.loads(info_path.read_text())
    info.update(total_episodes=1, total_frames=3)
    info_path.write_text(json.dumps(info))
    (root / "meta/tasks.parquet").touch()
    episodes = root / "meta/episodes/chunk-000"
    episodes.mkdir(parents=True)
    (episodes / "file-000.parquet").touch()
    calls = []

    def resume(**kwargs):
        calls.append(kwargs)
        return dataset

    monkeypatch.setattr(recorder.LeRobotDataset, "resume", resume)
    restored, cameras, resumed = create(root)
    assert restored is dataset and resumed and cameras == ["cam_left"]
    assert calls == [{"repo_id": "local/restart-test", "root": root, "vcodec": "h264"}]
