from PIL import Image as im
import random
import numpy as np

shelves = set()
# size of the warehosue floor
size = (100, 100)
# size of a block of shelves
block_size = (2,10)
# number of blocks of shelves
num_of_shelf_clusters = 10

'''Add a shelf block to the warehouse map. returns
False if the new shelf resulted in a collision and was
unable to add to the map'''
def add_shelf():
    global shelves
    seedx = random.randint(0, size[0]-block_size[0])
    seedy = random.randint(0, size[1]-block_size[1])
    seed = (seedx, seedy)
    temp_shelf = set()
    for i in range(0,block_size[0]):
        for j in range(0,block_size[1]):
            temp_shelf.add((seed[0]+i, seed[1]+j))

    #check for collision
    added_count = len(temp_shelf)
    original_count = len(shelves)
    temp = temp_shelf.union(shelves)
    if (len(temp) != added_count + original_count):
        return False
    else:
        shelves = temp
        return True

# add shelves
for i in range(num_of_shelf_clusters):
    success = False
    while not success:
        success = add_shelf()

# generate image
ary = np.ones((size[0], size[1], 3), dtype='uint8') * 255
for pt in shelves:
    ary[pt[0],pt[1]] = np.array([0,0,0])

foo = im.fromarray(ary, mode='RGB')
foo.save('foo.png')

