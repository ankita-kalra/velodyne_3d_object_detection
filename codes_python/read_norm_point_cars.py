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
parser.add_argument('--data_dir', default='D://Capstone//pcl_code//correspondence_grouping//build//Release//', type=str, help='path to the directory containing all data')
parser.add_argument('--out_dir', default='D://Capstone//pcl_code//correspondence_grouping//build//Release//', type=str)
parser.add_argument('--sample_ratio', default=1000,type=int)
parser.add_argument('--car_sample_ratio', default=1, type=int)
parser.add_argument('--site_name', default='domfountain_station1', type=str)
parser.add_argument('--scale', default=30.0, type=float, help='scale ratio for scene')
parser.add_argument('--trans_x1', default=0.0, type=float)
parser.add_argument('--trans_y1', default=0.0, type=float)
parser.add_argument('--trans_z1', default=0.0, type=float)
parser.add_argument('--car_name1', default='car1', type=str)
parser.add_argument('--trans_x2', default=1.0, type=float)
parser.add_argument('--trans_y2', default=10.2, type=float)
parser.add_argument('--trans_z2', default=0.1, type=float)
parser.add_argument('--car_name2', default='car2', type=str)
parser.add_argument('--trans_x3', default=-1.0, type=float)
parser.add_argument('--trans_y3', default=5.2, type=float)
parser.add_argument('--trans_z3', default=-0.1, type=float)
parser.add_argument('--car_name3', default='car3', type=str)
parser.add_argument('--car_r', default=255, type=int)
parser.add_argument('--car_g', default=255, type=int)
parser.add_argument('--car_b', default=0, type=int)
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
#trans_ = [args.trans_x, args.trans_y, args.trans_z]
scale = args.scale
car_name1 = args.car_name1
car_name2 = args.car_name2
car_name3 = args.car_name3
bmw_npy1 = '{}.npy'.format(car_name1)
bmw_npy2 = '{}.npy'.format(car_name2)
bmw_npy3 = '{}.npy'.format(car_name3)
car_sample_ratio = args.car_sample_ratio

## output names
out_point_path = os.path.join(out_dir, site_name+'_xyz_rgb_{}.pcd'.format(sample_ratio))
out_point_path_norm = os.path.join(out_dir, site_name+'_xyz_rgb_{}_normtrans_3car_{}.pcd'.format(sample_ratio,car_sample_ratio))
print(out_point_path_norm)
out_hyper_path = os.path.join(out_dir, site_name+'_hyper_{}.txt'.format(sample_ratio))
out_car_path1 = os.path.join(out_dir, '{}.pcd'.format(car_name1))
out_car_path_norm1 = os.path.join(out_dir, '{}_{}.pcd'.format(car_name1, car_sample_ratio))
print(out_car_path_norm1)
out_car_path2 = os.path.join(out_dir, '{}.pcd'.format(car_name2))
out_car_path_norm2 = os.path.join(out_dir, '{}_{}.pcd'.format(car_name2, car_sample_ratio))
print(out_car_path_norm2)
out_car_path3 = os.path.join(out_dir, '{}.pcd'.format(car_name3))
out_car_path_norm3 = os.path.join(out_dir, '{}_{}.pcd'.format(car_name3, car_sample_ratio))
print(out_car_path_norm3)
with open(point_file_path, 'r') as f:
    points = f.readlines()

sample_points = points[::sample_ratio]
debug_p = sample_points[0]

po = np.fromstring(debug_p, dtype=np.float32, sep=' ')
##print po

mean_point = np.zeros((3,), dtype=np.float32)
max_rad = 0
idx = 0

## scene
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

benz_points1 = np.load(bmw_npy1)
benz_points1 = benz_points1[:, (2,0,1)]
mean_benz1 = np.mean(benz_points1, axis=0)
benz_centered1 = benz_points1 - np.tile(mean_benz1, (benz_points1.shape[0], 1))
benz_norm1 = benz_centered1 / np.max(abs(benz_centered1[:]))
benz_points2 = np.load(bmw_npy2)
benz_points2 = benz_points2[:, (2,0,1)]
mean_benz2 = np.mean(benz_points2, axis=0)
benz_centered2 = benz_points2 - np.tile(mean_benz2, (benz_points2.shape[0], 1))
benz_norm2 = benz_centered2 / np.max(abs(benz_centered2[:]))
benz_points3 = np.load(bmw_npy3)
benz_points3 = benz_points3[:, (2,0,1)]
mean_benz3 = np.mean(benz_points3, axis=0)
benz_centered3 = benz_points3 - np.tile(mean_benz3, (benz_points3.shape[0], 1))
benz_norm3 = benz_centered3 / np.max(abs(benz_centered3[:]))
car_sample_points1 = benz_norm1[::car_sample_ratio]
car_sample_points2 = benz_norm2[::car_sample_ratio]
car_sample_points3 = benz_norm3[::car_sample_ratio]
#print mean_benz
#print np.max(benz_norm[:])
#print benz_norm[0, ...]
car_rgb = [args.car_r, args.car_g, args.car_b]

## scene with car
with open(out_point_path_norm, 'w') as of:
    # write header
    of.write('# .PCD v.7 - Point Cloud Data file format\r\n')
    of.write('VERSION .7\r\n')
    of.write('FIELDS x y z rgb\r\n')
    of.write('SIZE 4 4 4 4\r\n')
    of.write('TYPE F F F F\r\n')
    of.write('COUNT 1 1 1 1\r\n')
    of.write('WIDTH {}\r\n'.format(len(sample_points)+car_sample_points1.shape[0]+car_sample_points2.shape[0]+car_sample_points3.shape[0]))
    of.write('HEIGHT 1\r\n')
    of.write('VIEWPOINT 0 0 0 1 0 0 0\r\n')
    of.write('POINTS {}\r\n'.format(len(sample_points)+car_sample_points1.shape[0]+car_sample_points2.shape[0]+car_sample_points3.shape[0]))
    of.write('DATA ascii\r\n')
    for line in sample_points:
        point = np.fromstring(line, dtype=np.float32, sep=' ')
        coor = point[0:3]
        rgb = np.uint8(point[4:])
        idx += 1
        line_out = '{:.3f} {:.3f} {:.3f} {}\r\n'.format((coor[0]-mean_point[0])*scale/max_rad,
            (coor[1]-mean_point[1])*scale/max_rad,
            (coor[2]-mean_point[2])*scale/max_rad, rgb[2]<<16|rgb[1]<<8|rgb[0])
        of.write(line_out)

    for point in car_sample_points1:
        line_out = '{:.3f} {:.3f} {:.3f} {}\r\n'.format(point[0]+args.trans_x1,point[1]+args.trans_y1,point[2]+args.trans_z1, car_rgb[2]<<16|car_rgb[1]<<8|car_rgb[0])
        of.write(line_out)

    for point in car_sample_points2:
        line_out = '{:.3f} {:.3f} {:.3f} {}\r\n'.format(point[0]+args.trans_x2,point[1]+args.trans_y2,point[2]+args.trans_z2, car_rgb[2]<<16|car_rgb[1]<<8|car_rgb[0])
        of.write(line_out)

    for point in car_sample_points3:
        line_out = '{:.3f} {:.3f} {:.3f} {}\r\n'.format(point[0]+args.trans_x3,point[1]+args.trans_y3,point[2]+args.trans_z3, car_rgb[2]<<16|car_rgb[1]<<8|car_rgb[0])
        of.write(line_out)

## car 1
with open(out_car_path1, 'w') as of:
    # write header
    of.write('# .PCD v.7 - Point Cloud Data file format\r\n')
    of.write('VERSION .7\r\n')
    of.write('FIELDS x y z rgb\r\n')
    of.write('SIZE 4 4 4 4\r\n')
    of.write('TYPE F F F F\r\n')
    of.write('COUNT 1 1 1 1\r\n')
    of.write('WIDTH {}\r\n'.format(benz_centered1.shape[0]))
    of.write('HEIGHT 1\r\n')
    of.write('VIEWPOINT 0 0 0 1 0 0 0\r\n')
    of.write('POINTS {}\r\n'.format(benz_centered1.shape[0]))
    of.write('DATA ascii\r\n')
    for point in benz_norm1:
        line_out = '{:.3f} {:.3f} {:.3f} {}\r\n'.format(point[0],point[1],point[2], car_rgb[2]<<16|car_rgb[1]<<8|car_rgb[0])
        of.write(line_out)



with open(out_car_path_norm1, 'w') as of:
    # write header
    of.write('# .PCD v.7 - Point Cloud Data file format\r\n')
    of.write('VERSION .7\r\n')
    of.write('FIELDS x y z rgb\r\n')
    of.write('SIZE 4 4 4 4\r\n')
    of.write('TYPE F F F F\r\n')
    of.write('COUNT 1 1 1 1\r\n')
    of.write('WIDTH {}\r\n'.format(car_sample_points1.shape[0]))
    of.write('HEIGHT 1\r\n')
    of.write('VIEWPOINT 0 0 0 1 0 0 0\r\n')
    of.write('POINTS {}\r\n'.format(car_sample_points1.shape[0]))
    of.write('DATA ascii\r\n')
    for point in car_sample_points1:
        line_out = '{:.3f} {:.3f} {:.3f} {}\r\n'.format(point[0],point[1],point[2], car_rgb[2]<<16|car_rgb[1]<<8|car_rgb[0])
        of.write(line_out)

## car 2
with open(out_car_path2, 'w') as of:
    # write header
    of.write('# .PCD v.7 - Point Cloud Data file format\r\n')
    of.write('VERSION .7\r\n')
    of.write('FIELDS x y z rgb\r\n')
    of.write('SIZE 4 4 4 4\r\n')
    of.write('TYPE F F F F\r\n')
    of.write('COUNT 1 1 1 1\r\n')
    of.write('WIDTH {}\r\n'.format(benz_centered2.shape[0]))
    of.write('HEIGHT 1\r\n')
    of.write('VIEWPOINT 0 0 0 1 0 0 0\r\n')
    of.write('POINTS {}\r\n'.format(benz_centered2.shape[0]))
    of.write('DATA ascii\r\n')
    for point in benz_norm2:
        line_out = '{:.3f} {:.3f} {:.3f} {}\r\n'.format(point[0],point[1],point[2], car_rgb[2]<<16|car_rgb[1]<<8|car_rgb[0])
        of.write(line_out)



with open(out_car_path_norm2, 'w') as of:
    # write header
    of.write('# .PCD v.7 - Point Cloud Data file format\r\n')
    of.write('VERSION .7\r\n')
    of.write('FIELDS x y z rgb\r\n')
    of.write('SIZE 4 4 4 4\r\n')
    of.write('TYPE F F F F\r\n')
    of.write('COUNT 1 1 1 1\r\n')
    of.write('WIDTH {}\r\n'.format(car_sample_points2.shape[0]))
    of.write('HEIGHT 1\r\n')
    of.write('VIEWPOINT 0 0 0 1 0 0 0\r\n')
    of.write('POINTS {}\r\n'.format(car_sample_points2.shape[0]))
    of.write('DATA ascii\r\n')
    for point in car_sample_points2:
        line_out = '{:.3f} {:.3f} {:.3f} {}\r\n'.format(point[0],point[1],point[2], car_rgb[2]<<16|car_rgb[1]<<8|car_rgb[0])
        of.write(line_out)

## car 3
with open(out_car_path3, 'w') as of:
    # write header
    of.write('# .PCD v.7 - Point Cloud Data file format\r\n')
    of.write('VERSION .7\r\n')
    of.write('FIELDS x y z rgb\r\n')
    of.write('SIZE 4 4 4 4\r\n')
    of.write('TYPE F F F F\r\n')
    of.write('COUNT 1 1 1 1\r\n')
    of.write('WIDTH {}\r\n'.format(benz_centered3.shape[0]))
    of.write('HEIGHT 1\r\n')
    of.write('VIEWPOINT 0 0 0 1 0 0 0\r\n')
    of.write('POINTS {}\r\n'.format(benz_centered3.shape[0]))
    of.write('DATA ascii\r\n')
    for point in benz_norm3:
        line_out = '{:.3f} {:.3f} {:.3f} {}\r\n'.format(point[0],point[1],point[2], car_rgb[2]<<16|car_rgb[1]<<8|car_rgb[0])
        of.write(line_out)



with open(out_car_path_norm3, 'w') as of:
    # write header
    of.write('# .PCD v.7 - Point Cloud Data file format\r\n')
    of.write('VERSION .7\r\n')
    of.write('FIELDS x y z rgb\r\n')
    of.write('SIZE 4 4 4 4\r\n')
    of.write('TYPE F F F F\r\n')
    of.write('COUNT 1 1 1 1\r\n')
    of.write('WIDTH {}\r\n'.format(car_sample_points3.shape[0]))
    of.write('HEIGHT 1\r\n')
    of.write('VIEWPOINT 0 0 0 1 0 0 0\r\n')
    of.write('POINTS {}\r\n'.format(car_sample_points3.shape[0]))
    of.write('DATA ascii\r\n')
    for point in car_sample_points3:
        line_out = '{:.3f} {:.3f} {:.3f} {}\r\n'.format(point[0],point[1],point[2], car_rgb[2]<<16|car_rgb[1]<<8|car_rgb[0])
        of.write(line_out)
