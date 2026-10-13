from .pose import pose_rotation, vector6


class ActionDecoder:
    """Decode metre / radian deltas in a declared base or tool frame.

    Rotation deltas are ZYX Euler rotations, not component-wise Euler offsets.
    Base: p' = p + dp, R' = dR @ R.
    Tool: p' = p + R @ dp, R' = R @ dR.
    """

    def __init__(self, frame="base"):
        if frame not in {"base", "tool"}:
            raise ValueError("frame must be 'base' or 'tool'")
        self.frame = frame

    def delta_to_pose(self, current_pose, delta_action):
        current = vector6(current_pose, "current_pose")
        delta = vector6(delta_action, "delta_action")
        rotation = pose_rotation(current)
        target = current.copy()
        if self.frame == "base":
            target[:3] += delta[:3]
            target_rotation = pose_rotation(delta) * rotation
        else:
            target[:3] += rotation.apply(delta[:3])
            target_rotation = rotation * pose_rotation(delta)
        # Preserve the controller's Euler branch for pure translations.
        if any(delta[3:]):
            target[3:] = target_rotation.as_euler("xyz")
        return target.tolist()
