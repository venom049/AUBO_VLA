import numpy as np


class ActionDecoder:

    def delta_to_pose(self,
                      current_pose,
                      delta_action):

        """
        Convert VLA delta action
        into AUBO target pose
        """

        return (
            np.array(current_pose)
            +
            np.array(delta_action)
        ).tolist()
