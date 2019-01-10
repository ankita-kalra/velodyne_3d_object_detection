#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_cloud.h>
#include <pcl/console/parse.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <iostream>

#define PCL_NO_PRECOMPILE

typedef pcl::PointXYZRGB PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::SHOT352 DescriptorType;

std::string model_filename_;
std::string scene_filename_;

bool show_keypoints_(false);
bool show_correspondences_(false);
bool use_cloud_resolution_(false);
bool use_hough_(true);
float model_ss_(0.0005f);
float scene_ss_(0.005f);
float rf_rad_(0.015f);
float descr_rad_(0.02f);
float cg_size_(0.01f);
float cg_thresh_(5.0f);
float x(0.0f);
float y(0.0f);
float z(0.0f);

void
parseCommandLine(int argc, char *argv[])
{
	
	//Model & scene filenames
	std::vector<int> filenames;
	filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
	if (filenames.size() != 2)
	{
		std::cout << "Filenames missing.\n";
		exit(-1);
	}

	model_filename_ = argv[filenames[0]];
	scene_filename_ = argv[filenames[1]];

	pcl::console::parse_argument(argc, argv, "--x", x);
	pcl::console::parse_argument(argc, argv, "--y", y);
	pcl::console::parse_argument(argc, argv, "--z", z);


}


boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
	// --------------------------------------------
	// -----Open 3D viewer and add point cloud-----
	// --------------------------------------------
	boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
	viewer->setBackgroundColor(0, 0, 0);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
	viewer->addPointCloud<pcl::PointXYZRGB>(cloud, rgb, "sample cloud");
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer->addCoordinateSystem(1.0);
	viewer->initCameraParameters();
	return (viewer);
}

float get_mean_distance(pcl::PointCloud<PointType>::Ptr inputCloud)
{
	int totalcount = inputCloud->width * inputCloud->height;
	pcl::PointCloud<PointType>::Ptr input_keypoints(new pcl::PointCloud<PointType>());
	pcl::PointCloud<NormalType>::Ptr input_normals(new pcl::PointCloud<NormalType>());
	pcl::NormalEstimationOMP<PointType, NormalType> norm_est;
	norm_est.setKSearch(10);
	norm_est.setInputCloud(inputCloud);
	norm_est.compute(*input_normals);
	pcl::UniformSampling<PointType> uniform_sampling;
	pcl::PointCloud<DescriptorType>::Ptr input_descriptors(new pcl::PointCloud<DescriptorType>());
	uniform_sampling.setInputCloud(inputCloud);
	uniform_sampling.setRadiusSearch(model_ss_);
	uniform_sampling.filter(*input_keypoints);
	pcl::SHOTEstimationOMP<PointType, NormalType, DescriptorType> descr_est;
	descr_est.setRadiusSearch(descr_rad_);
	descr_est.setInputCloud(input_keypoints);
	descr_est.setInputNormals(input_normals);
	descr_est.setSearchSurface(inputCloud);
	descr_est.compute(*input_descriptors);
	cout << "descriptor computed" << endl;
	float *EuclidianDistance = new float[totalcount];

	pcl::KdTreeFLANN<DescriptorType> kdtree;
	kdtree.setInputCloud(input_descriptors);

	int K = 2; //first will be the distance with point it self and second will the nearest point that's why "2"

	std::vector<int> pointIdxNKNSearch(K);
	std::vector<float> pointNKNSquaredDistance(K);

	for (int i = 0; i < totalcount; ++i)
	{
		std::cout << "\nK nearest neighbor search at (" << inputCloud->points[i].x
			<< " " << inputCloud->points[i].y
			<< " " << inputCloud->points[i].z
			<< ") with K=" << K << std::endl;

		if (kdtree.nearestKSearch(i, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			for (size_t j = 0; j < pointIdxNKNSearch.size(); ++j)
			{
				//saving all the distance in Vector
				EuclidianDistance[i] = pointNKNSquaredDistance[j];

			}
		}
	}
	float totalDistance = 0.0,meanDistance=0.0;
	for (int i = 0; i < totalcount; i++)
	{
		//accumulating all distances
		totalDistance = totalDistance + EuclidianDistance[i];
	}

	//calculating the mean distance
	meanDistance = totalDistance / totalcount;

	//freeing the allocated memory      
	delete[] EuclidianDistance;

}


int
main(int argc, char** argv)
{

	parseCommandLine(argc, argv);

	pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr model_output(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene_output(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());
	pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType>());
	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(new pcl::PointCloud<DescriptorType>());

	pcl::PointCloud<PointType> scene_c;

	clock_t begin = clock();

	//
	//  Load clouds
	//
	
	if (pcl::io::loadPCDFile(model_filename_, *model) < 0)
	{
		std::cout << "Error loading model cloud." << std::endl;
		//showHelp(argv[0]);
		return (-1);
	}
	clock_t end = clock();
	double elapsed_secs = double(end - begin) / CLOCKS_PER_SEC;

	cout << "Elapsed Time: " << elapsed_secs << endl;
	if (pcl::io::loadPCDFile(scene_filename_, *scene) < 0)
	{
		std::cout << "Error loading scene cloud." << std::endl;
		//showHelp(argv[0]);
		return (-1);
	}

	//cout << "get_mean_distance(model): " << get_mean_distance(model) << endl;
	//cout << "get_mean_distance(scene): " << get_mean_distance(scene) << endl;
	   scene_c = *model;
		float theta = M_PI / 2;

		Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();

		// Define a translation of 2.5 meters on the x axis.
		transform_2.translation() << x, y, z;

		// The same rotation matrix as before; theta radians around Z axis
		transform_2.rotate(Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ()));

		// Print the transformation
		printf("\nMethod #2: using an Affine3f\n");
		std::cout << transform_2.matrix() << std::endl;

	    float sum_x = 0, sum_y = 0, sum_z = 0;
		const float bad_point = std::numeric_limits<float>::quiet_NaN();
		cout << "Model size before NaNs: " << model->points.size() << endl;
		std::vector<int> indices;
		pcl::removeNaNFromPointCloud(*model, *model_output, indices);
		std::cout << "Model size after removing NaNs: " << model_output->points.size() << std::endl;



		for (int i = 0; i < model_output->points.size(); i++)
		{
			if (model_output->points[i].x != bad_point)
			{  
				//cout << sum_x + model->points[i].x;
				sum_x = sum_x + model_output->points[i].x;
				sum_y = sum_y + model_output->points[i].y;
				sum_z = sum_z + model_output->points[i].z;
			}
			
		}
		//cout << "Model size after NaNs: " << model_output->points.size() << endl;
		cout << "Centroid for model: " << sum_x / model->points.size() << " " << sum_y / model->points.size() << " " << sum_z / model->points.size() << endl;
		sum_x = 0, sum_y = 0, sum_z = 0;
		pcl::removeNaNFromPointCloud(*scene, *scene_output, indices);
 		for (int i = 0; i < scene_output->points.size(); i++)
		{
			sum_x = sum_x + scene_output->points[i].x;
			sum_y = sum_y + scene_output->points[i].y;
			sum_z = sum_z + scene_output->points[i].z;
		}
		cout << "Centroid for scene : " << sum_x/scene_output->points.size() << " " << sum_y/scene_output->points.size() << " " << sum_z/scene_output->points.size() << endl;
		// Executing the transformation
		pcl::PointCloud<PointType>::Ptr transformed_cloud(new pcl::PointCloud<PointType>());
		// You can either apply transform_1 or transform_2; they are the same
		//pcl::transformPointCloud(*scene, *transformed_cloud, transform_2);
		transformed_cloud = scene_output;




		for (size_t i = 0; i < scene_output->points.size(); ++i)
		{   
			//cout << "transformed_cloud->points[1].x" << transformed_cloud->points[1].x << endl;
			transformed_cloud->points[i].x += x;
			//cout << "transformed_cloud->points[1].x" << transformed_cloud->points[1].x << endl;
			transformed_cloud->points[i].y += y;
			transformed_cloud->points[i].z += z;
		}
		

		scene_c += *transformed_cloud;

		
		std::cerr << "saving Cloud C: " << std::endl;
		
		pcl::io::savePCDFileASCII("test_pcd.pcd", scene_c);
		std::cerr << "Saved " << scene_c.points.size() << " data points to test_pcd.pcd." << std::endl;

		pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
		ne.setInputCloud(transformed_cloud);
		pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());
		ne.setSearchMethod(tree);
		pcl::PointCloud<pcl::Normal>::Ptr normals;
		ne.setRadiusSearch(0.05);
		ne.compute(*normals);



		pcl::PointCloud<PointType>::Ptr scene_c_ptr(&scene_c);
		pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
		viewer.addPointCloud(scene_c_ptr, "scene_c");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "scene_c");
		viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal>(transformed_cloud, normals, 10, 0.05, "normals");
		viewer.addCoordinateSystem(1.0);



		while (!viewer.wasStopped())
		{
			viewer.spinOnce();
		}


	return (0);
}