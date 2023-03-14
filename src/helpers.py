import numpy as np

# create a global list of possible
# RGB values
colors = []
for r in [0, 255]:
    for g in [0, 255]:
        for b in [0, 255]:
            colors.append((r,g,b))

colors.remove((0,0,0))
colors.remove((255,255,255))

# This class is just for painting robot
# paths on the png in different colors
class Painter:
    def __init__(self, color, map) -> None:
        self.color = color
        self.ary_map = map

    def paint_move(self, pt):
        self.ary_map[pt[0], pt[1]] = np.array(self.color)

# a class to store path info
# when subscribed to the robot obj
class RobotPath:
    def __init__(self, name) -> None:
        self.name = name
        self._path = []

    def add_to_path(self, pt):
        self._path.append(pt)

    @property
    def path(self):
        return self._path

# simple counter that subscribes
# to the deadlock event
class Counter:
    def __init__(self) -> None:
        self._count = 0

    def increment(self):
        self._count = self._count + 1

    @property
    def count(self):
        return self._count

def write_line_to_file(filepath, array, open_mode="a"):
    with open(filepath, open_mode) as f:
        f.write(",".join(array))
        f.write("\n")
