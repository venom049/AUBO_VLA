class RobotAction:
    """
    Unified robot action interface.

    VLA output:

    arm:
    dx dy dz dRx dRy dRz

    gripper:
    end-effector command
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
