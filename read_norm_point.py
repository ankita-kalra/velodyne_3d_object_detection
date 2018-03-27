import os
import sys
import numpy as np 
import scipy.io as sio
import argparse
####
# TODO:
# README:
# to change the translation and scale of scene, just modify variable scale and tuple trans_
# the ouput names are defined at output name section
#   out_car_path is the name for car pcd
#   out_hyper_path is the name for hyperparameters of scene
#   out_point_path_norm is the name for concatenation of car and scene
#   out_point_path is for the scene pcd file
####
parser = argparse.ArgumentParser()
parser.add_argument('--data_dir', default='./', type=str, help='path to the directory containing all data')
parser.add_argument('--out_dir', default='./', type=str)
parser.add_argument('--sample_ratio', default=10000, type=int)
parser.add_argument('--car_sample_ratio', default=1000, type=int)
parser.add_argument('--site_name', default='bildstein_station3', type=str)
parser.add_argument('--scale', default=1.0, type=float, help='scale ratio for scene')
parser.add_argument('--trans_x', default=0.0, type=float)
parser.add_argument('--trans_y', default=0.0, type=float)
parser.add_argument('--trans_z', default=0.0, type=float)
parser.add_argument('--car_name', default='bmw_x5', type=str)
parser.add_argument('--car_r', default=255, type=int)
parser.add_argument('--car_g', default=255, type=int)
parser.add_argument('--car_b', default=255, type=int)
args = parser.parse_args()

#site_name = 'bildstein_station3'
#data_dir = './'
#out_dir = './'
#point_file_path = os.path.join(data_dir, site_name+'_xyz_intensity_rgb.txt')
#sample_ratio = 100000
#trans_ = [0.0, 0.0, 0.0]
#scale = 1.0
#bmw_npy = 'bmw_x5.npy'

site_name = args.site_name
data_dir = args.data_dir
out_dir = args.out_dir
point_file_path = os.path.join(data_dir, site_name+'_xyz_intensity_rgb.txt')
sample_ratio = args.sample_ratio
trans_ = [args.trans_x, args.trans_y, args.trans_z]
scale = args.scale
car_name = args.car_name
bmw_npy = '{}.npy'.format(car_name)
car_sample_ratio = args.car_sample_ratio

## output names
out_point_path = os.path.join(out_dir, site_name+'_xyz_rgb_{}.pcd'.format(sample_ratio))
out_point_path_norm = os.path.join(out_dir, site_name+'_xyz_rgb_{}_norm.pcd'.format(sample_ratio))
out_hyper_path = os.path.join(out_dir, site_name+'_hyper_{}.txt'.format(sample_ratio))
out_car_path = os.path.join(out_dir, '{}.pcd'.format(car_name))
out_car_path_norm = os.path.join(out_dir, '{}_{}.pcd'.format(car_name, car_sample_ratio))

with open(point_file_path, 'r') as f:
    points = f.readlines()

sample_points = points[::sample_ratio]
debug_p = sample_points[0]

po = np.fromstring(debug_p, dtype=np.float32, sep=' ')
##print po

mean_point = np.zeros((3,), dtype=np.float32)
max_rad = 0
idx = 0

with open(out_point_path, 'w') as of:
    # write header
    of.write('# .PCD v.7 - Point Cloud Data file format\r\n')
    of.write('VERSION .7\r\n')
    of.write('FIELDS x y z rgb\r\n')
    of.write('SIZE 4 4 4 4\r\n')
    of.write('TYPE F F F F\r\n')
    of.write('COUNT 1 1 1 1\r\n')
    of.write('WIDTH {}\r\n'.format(len(sample_points)))
    of.write('HEIGHT 1\r\n')
    of.write('VIEWPOINT 0 0 0 1 0 0 0\r\n')
    of.write('POINTS {}\r\n'.format(len(sample_points)))
    of.write('DATA ascii\r\n')
    for line in sample_points:
        point = np.fromstring(line, dtype=np.float32, sep=' ')
        coor = point[0:3]
        rgb = np.uint8(point[4:])
        mean_point = (coor + mean_point*idx)/(idx+1)
        idx += 1
        line_out = '{:3f} {:3f} {:3f} {}\r\n'.format(coor[0], coor[1], coor[2], rgb[2]<<16|rgb[1]<<8|rgb[0])
        of.write(line_out)

for line in sample_points:
    point = np.fromstring(line, dtype=np.float32, sep=' ')
    coor = point[0:3]
    max_rad = np.maximum(max_rad, np.linalg.norm(coor-mean_point)) 

#print mean_point
#print max_rad

with open(out_hyper_path, 'w') as of:
    line_out = '{} {} {} {}\r\n'.format(mean_point[0], mean_point[1], mean_point[2], max_rad)
    of.write(line_out)

benz_points = np.load(bmw_npy)
mean_benz = np.mean(benz_points, axis=0)
benz_centered = benz_points - np.tile(mean_benz, (benz_points.shape[0], 1))
benz_norm = benz_centered / np.max(abs(benz_centered[:]))
#print mean_benz
#print np.max(benz_norm[:])
#print benz_norm[0, ...]
car_rgb = [args.car_r, args.car_g, args.car_b]

with open(out_point_path_norm, 'w') as of:
    # write header
    of.write('# .PCD v.7 - Point Cloud Data file format\r\n')
    of.write('VERSION .7\r\n')
    of.write('FIELDS x y z rgb\r\n')
    of.write('SIZE 4 4 4 4\r\n')
    of.write('TYPE F F F F\r\n')
    of.write('COUNT 1 1 1 1\r\n')
    of.write('WIDTH {}\r\n'.format(len(sample_points)+benz_centered.shape[0]))
    of.write('HEIGHT 1\r\n')
    of.write('VIEWPOINT 0 0 0 1 0 0 0\r\n')
    of.write('POINTS {}\r\n'.format(len(sample_points)+benz_centered.shape[0]))
    of.write('DATA ascii\r\n')
    for line in sample_points:
        point = np.fromstring(line, dtype=np.float32, sep=' ')
        coor = point[0:3]
        rgb = np.uint8(point[4:])
        idx += 1
        line_out = '{:.3f} {:.3f} {:.3f} {}\r\n'.format((coor[0]-mean_point[0]+trans_[0])*scale/max_rad,
            (coor[1]-mean_point[1]+trans_[1])*scale/max_rad,
            (coor[2]-mean_point[2]+trans_[2])*scale/max_rad, rgb[2]<<16|rgb[1]<<8|rgb[0])
        of.write(line_out)

    for point in benz_norm:
        line_out = '{:.3f} {:.3f} {:.3f} {}\r\n'.format(point[0],point[1],point[2], car_rgb[2]<<16|car_rgb[1]<<8|car_rgb[0])
        of.write(line_out)

with open(out_car_path, 'w') as of:
    # write header
    of.write('# .PCD v.7 - Point Cloud Data file format\r\n')
    of.write('VERSION .7\r\n')
    of.write('FIELDS x y z rgb\r\n')
    of.write('SIZE 4 4 4 4\r\n')
    of.write('TYPE F F F F\r\n')
    of.write('COUNT 1 1 1 1\r\n')
    of.write('WIDTH {}\r\n'.format(benz_centered.shape[0]))
    of.write('HEIGHT 1\r\n')
    of.write('VIEWPOINT 0 0 0 1 0 0 0\r\n')
    of.write('POINTS {}\r\n'.format(benz_centered.shape[0]))
    of.write('DATA ascii\r\n')
    for point in benz_norm:
        line_out = '{:.3f} {:.3f} {:.3f} {}\r\n'.format(point[0],point[1],point[2], car_rgb[2]<<16|car_rgb[1]<<8|car_rgb[0])
        of.write(line_out)


car_sample_points = benz_norm[::car_sample_ratio]
with open(out_car_path_norm, 'w') as of:
    # write header
    of.write('# .PCD v.7 - Point Cloud Data file format\r\n')
    of.write('VERSION .7\r\n')
    of.write('FIELDS x y z rgb\r\n')
    of.write('SIZE 4 4 4 4\r\n')
    of.write('TYPE F F F F\r\n')
    of.write('COUNT 1 1 1 1\r\n')
    of.write('WIDTH {}\r\n'.format(car_sample_points.shape[0]))
    of.write('HEIGHT 1\r\n')
    of.write('VIEWPOINT 0 0 0 1 0 0 0\r\n')
    of.write('POINTS {}\r\n'.format(car_sample_points.shape[0]))
    of.write('DATA ascii\r\n')
    for point in car_sample_points:
        line_out = '{:.3f} {:.3f} {:.3f} {}\r\n'.format(point[0],point[1],point[2], car_rgb[2]<<16|car_rgb[1]<<8|car_rgb[0])
        of.write(line_out)
