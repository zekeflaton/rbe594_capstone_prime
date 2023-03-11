from PIL import Image as im
import pandas as pd
import numpy as np


def generate_warehouse_numpy_map(map_file='../src/warehouse.csv'):
    csv_map = pd.read_csv(map_file, header=None).to_numpy()
    ary_map = np.ones((csv_map.shape[0], csv_map.shape[1], 3), dtype='uint8') * 255
    for i in range(csv_map.shape[0]):
        for j in range(csv_map.shape[1]):
            if csv_map[i, j] == 255:
                ary_map[i, j] = np.array([0, 0, 0])
    return ary_map


if __name__ == "__main__":
    warehouse_map = generate_warehouse_numpy_map()
    foo = im.fromarray(warehouse_map, mode='RGB')
    foo.save('warehouse_map.png')
