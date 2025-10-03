import numpy as np
import cv2
import matplotlib.pyplot as plt

def draw_shapes_cv2(grid, shapes):

    for shape_type, params in shapes.items():
        if shape_type == 'circle':
            x, y, r = params
            cv2.circle(grid, (x, y), r, color=1, thickness=-1)

        elif shape_type == 'rect':
            tx, ty, bx, by, thickness = params
            cv2.line(grid, (tx, ty), (bx, by), color=1, thickness=thickness)

    return grid

def display_grid(grid):
    plt.matshow(grid, cmap='gray_r')
    plt.axis('off')
    plt.show()

shapes = {
#     'circle': (30,30, 5),
#     'rect': (64, 64, 80, 70, 2),
 }

grid = np.load('../map/map.npy')
grid = draw_shapes_cv2(grid, shapes)

# modified_grid = draw_shapes_cv2(grid.copy(), shapes)  
display_grid(grid)
# save the new grid for development purposes
#np.save('../map/temp_map.npy', modified_grid) 