from control.action import RobotAction


class FakeVLA:

    def predict(self, observation=None):

        # simulate VLA output
        return RobotAction(
            dx=0.02,
            dy=0,
            dz=0,
            drx=0,
            dry=0,
            drz=0,
            gripper=0
        )


if __name__ == "__main__":

    model = FakeVLA()

    print(
        model.predict().vector()
    )
