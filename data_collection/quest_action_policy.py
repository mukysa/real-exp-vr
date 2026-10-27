"""Choose when a Quest collection sample may use measured joints as a hold target."""


def use_measured_hold(source_state: str | None, accepted_action_seen: bool) -> bool:
    """Allow zero-action observations only outside an active control phase.

    Before the first source status arrives, a bridge that has never received an
    accepted target may publish observations. Once control has begun, missing
    status must not hide a lost action stream.
    """
    if source_state in {"READY", "WAITING"}:
        return True
    if source_state in {"FOLLOWING", "RETURNING", "FAULT"}:
        return False
    return not accepted_action_seen
