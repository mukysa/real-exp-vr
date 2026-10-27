import sys

import pytest

from data_collection import reset_pylibfranka as reset


@pytest.mark.parametrize(
    ("options", "expected_processes", "expected_grippers"),
    [
        ([], ["left_arm_reset", "right_arm_reset"], ["Left", "Right"]),
        (["--left"], ["left_arm_reset"], ["Left"]),
        (["--right"], ["right_arm_reset"], ["Right"]),
        (["--left", "--skip-grippers"], ["left_arm_reset"], []),
    ],
)
def test_reset_starts_only_selected_hardware(monkeypatch, options, expected_processes, expected_grippers):
    process_names = []
    gripper_names = []

    class FakeProcess:
        def __init__(self, *, target, args, name):
            assert target is reset.arm_worker
            assert args[2] == name.split("_")[0].title() + " Arm"
            self.name = name

        def start(self):
            process_names.append(self.name)

        def join(self):
            pass

    monkeypatch.setattr(sys, "argv", ["reset_pylibfranka.py", *options])
    monkeypatch.setattr(reset.mp, "Process", FakeProcess)
    monkeypatch.setattr(reset, "move_gripper", lambda _ip, _width, name, _abort: gripper_names.append(name))

    reset.main()

    assert process_names == expected_processes
    assert gripper_names == expected_grippers


@pytest.mark.parametrize("side,other", [("--left", "Right"), ("--right", "Left")])
def test_dry_run_shows_only_selected_side(monkeypatch, capsys, side, other):
    monkeypatch.setattr(sys, "argv", ["reset_pylibfranka.py", side, "--dry-run"])
    monkeypatch.setattr(reset.mp, "Process", lambda **_: pytest.fail("started hardware process"))
    reset.main()
    output = capsys.readouterr().out
    assert other not in output
    assert ("Left" if side == "--left" else "Right") in output


def test_left_and_right_are_mutually_exclusive(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["reset_pylibfranka.py", "--left", "--right"])
    with pytest.raises(SystemExit) as exc:
        reset.parse_args()
    assert exc.value.code == 2
