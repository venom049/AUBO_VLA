class RobotAction:
    """
    Unified robot action interface.

    VLA output:

    arm:
    dx dy dz dRx dRy dRz (metres / ZYX Euler radians)
    The decoder declares whether deltas use the base or tool frame.

    gripper:
    Reserved end-effector command; the current arm driver does not execute it.
    """

    def __init__(self,
                 dx=0,
                 dy=0,
                 dz=0,
                 drx=0,
                 dry=0,
                 drz=0,
                 gripper=0):

        self.arm = [
            dx,
            dy,
            dz,
            drx,
            dry,
            drz
        ]

        self.gripper = gripper


    def vector(self):

        return self.arm + [
            self.gripper
        ]
