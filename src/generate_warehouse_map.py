from PIL import Image as im
import pandas as pd
import numpy as np


def generate_warehouse_numpy_map(map_file='../src/warehouse.csv'):
    """
    Uses the csv warehouse map file to generate a human friendly image

    :param str map_file: filepath for the map csv
    :return: np.ndarray ary_map: warehouse map in a numpy array
    """
    # Import the CSV as a numpy array
    csv_map = pd.read_csv(map_file, header=None).to_numpy()

    # Generate a 3D numpy array to save to an image later.  Default values are white (255, 255, 255)
    ary_map = np.ones((csv_map.shape[0], csv_map.shape[1], 3), dtype='uint8') * 255

    # For any location where there is a shelf turn that pixel black (0, 0, 0).
    # Charging stations are marked as red pixels
    for i in range(csv_map.shape[0]):
        for j in range(csv_map.shape[1]):
            if csv_map[i, j] == 255:
                ary_map[i, j] = np.array([0, 0, 0])
            if csv_map[i,j] == 1:
                ary_map[i, j] = np.array([255, 0, 0])
    return ary_map


if __name__ == "__main__":
    warehouse_map = generate_warehouse_numpy_map()
    foo = im.fromarray(warehouse_map, mode='RGB')
    foo.save('warehouse_map.png')
