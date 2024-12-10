# assembly.py
class Assembly:
    def __init__(self, cubes):
        self.cubes = cubes  # List of Cube objects

    def add_cube(self, cube):
        self.cubes.append(cube)

    def get_assembly_state(self):
        return [cube.get_state() for cube in self.cubes]
