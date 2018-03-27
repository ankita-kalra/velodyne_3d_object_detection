import os
import sys
import numpy as np 
import scipy.io as sio
import argparse

out_dir = './'
car_name = 'bmw_x5'
car_sample_ratio = 10
out_car_path_norm = os.path.join(out_dir, '{}_{}.pcd'.format(car_name, car_sample_ratio))

with open(out_car_path_norm, 'r') as f:
    point_f = f.readlines()
    points_lines = point_f[11:]
    #print points_lines[0]
    #print points_lines[0].split(' ')
    #print float(points_lines[0].split(' ')[0])
    #sys.exit()

points = []

for line in points_lines:
    temp_points = np.zeros((3,), dtype=np.float32)
    split_line = line.split(' ')
    for i in range(3):
        temp_points[i] = float(split_line[i])

    points.append(temp_points)

points = np.asarray(points, dtype=np.float32)

point_num = points.shape[0]
all_dis = []
for i, temp_point in enumerate(points):
    other_points = np.concatenate((points[0:i, ...], points[i+1:, ...]), axis=0) 
    distances = np.linalg.norm(other_points-np.tile(temp_point, (point_num-1, 1)), ord=2, axis=1)
    all_dis.append(min(distances))


print('minimum distance: {}'.format(min(all_dis)))
print('mean average min distance: {}'.format(np.mean(np.asarray(all_dis))))
