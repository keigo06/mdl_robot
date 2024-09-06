import numpy as np
import roboticstoolbox as rtb
from rospkg import RosPack


class ROBOT_MODEL_NAME(ERobot):

    def __init__(self):

        links, name, urdf_string, urdf_filepath = self.URDF_read(
            "robots/ROBOT_URDF.xacro", tld=RosPack().get_path('HARDWARE_PACKAGE_NAME') + '/data/xacro')

        super().__init__(
            links,
            name=name,
            urdf_string=urdf_string,
            urdf_filepath=urdf_filepath,
            manufacturer="ROBOT_MANUFACTURER_NAME",
            gripper_links=links[7]
        )

        self.addconfiguration(
            "qr", np.array([0, 0, 0, 0, 0, 0])
        )

    if __name__ == "__main__":  # pragma nocover

        robot = ROBOT_MODEL_NAME()
        print(robot)
