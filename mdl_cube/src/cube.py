# cube.py
class Cube:
    def __init__(self, cube_id, position, euler_angles):
        self.cube_id = cube_id
        self.position = position  # [x, y, z]
        self.euler_angles = euler_angles  # [roll, pitch, yaw]

    def update_position(self, new_position):
        self.position = new_position

    def update_orientation(self, new_euler_angles):
        self.euler_angles = new_euler_angles

    def get_state(self):
        return {
            'id': self.cube_id,
            'position': self.position,
            'euler_angles': self.euler_angles
        }
