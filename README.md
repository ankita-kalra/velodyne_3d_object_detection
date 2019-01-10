# Velodyne 3D Object Detection

## C++ Codes

The c++ codes can be compiled using Visual Studio 15 or Cmake, but requires the PCL library to be installed. I used the latest available version 1.7 for the same.

###### correspondence_grouping.cpp

This file implements the Geometric consistency method and the hough voting method for object detection.
The way to run this file and the arguments mean as follows-

###### Usage- 

```
executable_name model_filename.pcd scene_filename.pcd [Options]
```
 
###### descriptor_compares.cpp

This file performs normal objection detection by any of the above methods chosen by the above argument switches, however it has an extra argument "-desc" which can be switched to compare the performance of each descriptor for object detection and recognition. We found out the SHOT descriptor to be best in our setting.

###### objection_detection_modified.cpp

This file implements the Geometric consistency method and the hough voting method for object detection, however has modifications in the support descriptors and have bug fixes with respect to missing points in the point clouds and the executable from this code performs better than the correspondence_grouping.cpp.

###### Options-
	-h:  Show this help.
	-k:                     Show used keypoints.
	-c:                     Show used correspondences.
	-r:                     Compute the model cloud resolution and multiply.
	--algorithm (Hough|GC): Clustering algorithm used (default Hough).
	--model_ss val:         Model uniform sampling radius (default 0.01). 
	--scene_ss val:         Scene uniform sampling radius (default 0.03),
	--rf_rad val:           Reference frame radius (default 0.015). 
	--descr_rad val:        Descriptor radius (default 0.02). 
	--cg_size val:          Cluster size (default 0.01).
	--cg_thresh val:        Clustering threshold (default 5).
	--downsampling val:     enter the type of downsampling.

###### merge.cpp 

It was used for data prepartion for merging the point cloud of objects from the SHAPENET dataset with the objects from the Semantic 3D Dataset. This was done initially assuming that both the point cloud have the same viewpoint and are in the same scale, if this is not so then consider using python codes.

## Python Codes

The codes in python were mostly used for data prepartation and gives us general sense of parameters to be used in the above object detection code. The output filename using each of these filenames would have the parameters appended as underscores. 


###### Usage-

```
filename.py [Options]
```

###### Options-
```
'--data_dir' - 'path to the directory containing all data' default- "It is set to a path, in our server and can be modified"
'--out_dir' - 'the directory to store the output point cloud' - "It is set to a path, in our server and can be modified"
'--sample_ratio' - downsampling ratio of the point cloud default is 1000
'--car_sample_ratio' - downsampling ratio of the object, default is 1
'--site_name' - the scene name from semantic 3D Dataset, the default scene name was 'domfountain_station1'
'--scale' - Scale of the point cloud, it can alsp be thought of as resolution, but it is only for scene, default=30.0 
'--trans_x' - The position of object in x-direction, default=0.0
'--trans_y' - The position of object in y-direction, default=0.0
'--trans_z' - The position of object in z-direction, default=0.0
'--car_r' - decides the red intesity value of the points in object pasted onto the scene
'--car_g' - decides the green intesity value of the points in object pasted onto the scene
'--car_b' - decides the blue intesity value of the points in object pasted onto the scene
```

###### read_norm_point_cars.py

Since to experiment our methods we started of with chosing verious kind of cars as an object from Shapenet dataset, we tried to paste multiple cars on the scene point cloud, with different position in the x, y and z direction.

###### read_norm_point_cut.py

To test whether our detection algorithm is robust to mising point clouds, we tried to downsample the point cloud and as well as delete few percentage of the points from the original point cloud to compare the robustness. This file includes "-cut_ratio" as the parameter and we can generate varied dataset with respect to it.

###### read_norm_point_rot.py

We also tested the robustness of geometrical based object detection for various point cloud objects(eg. cars in the our initial experiments) at different orientation for testing.

###### compute_distance.py

This file computes the mean distance between the points in object point cloud and similarly does that for the scene point cloud, it was majorly done to get the support sizes for the descriptor to compute effective discriminative features.
 
