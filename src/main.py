from PIL import Image
import numpy as np
from obstacle_field_motion_planning import Orchestrator, RobotPathPlanner

png_map = Image.open('foo.png')
ary_map = np.array(png_map)
shelves = set()
size = ary_map.shape
for i in range(ary_map.shape[0]):
    for j in range(ary_map.shape[1]):
        px = ary_map[i,j]
        if px == 0:
            shelves.add((i,j))

orchestrator = Orchestrator(shelves, size)
start = (0,0)
end = shelves[0]
orchestrator.add_robot(1, start, end)
while not orchestrator.is_done():
    orchestrator.move_all()