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
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/usc.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/features/pfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/intensity_gradient.h>
#include <pcl/features/rift.h>
#include <pcl/features/vfh.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

typedef pcl::PointXYZ PointType;
typedef pcl::Normal NormalType;
typedef pcl::ReferenceFrame RFType;
typedef pcl::UniqueShapeContext1960 DescriptorType;
int v1(0);

std::string model_filename_;
std::string scene_filename_;

//Algorithm params
bool show_keypoints_(false);
bool show_correspondences_(false);
bool use_cloud_resolution_(false);
bool use_hough_(true);
bool use_vg_(false);
float model_ss_(0.01f);
float scene_ss_(0.03f);
float rf_rad_(0.015f);
float descr_rad_(0.02f);
float cg_size_(0.01f);
float cg_thresh_(5.0f);

void
showHelp(char *filename)
{
	std::cout << std::endl;
	std::cout << "***************************************************************************" << std::endl;
	std::cout << "*                                                                         *" << std::endl;
	std::cout << "*             Object Detection Wild Velodyne          *" << std::endl;
	std::cout << "*                                                                         *" << std::endl;
	std::cout << "***************************************************************************" << std::endl << std::endl;
	std::cout << "Usage: " << filename << " model_filename.pcd scene_filename.pcd [Options]" << std::endl << std::endl;
	std::cout << "Options:" << std::endl;
	std::cout << "     -h:                     Show this help." << std::endl;
	std::cout << "     -k:                     Show used keypoints." << std::endl;
	std::cout << "     -c:                     Show used correspondences." << std::endl;
	std::cout << "     -r:                     Compute the model cloud resolution and multiply" << std::endl;
	std::cout << "                             each radius given by that value." << std::endl;
	std::cout << "     --algorithm (Hough|GC): Clustering algorithm used (default Hough)." << std::endl;
	std::cout << "     --model_ss val:         Model uniform sampling radius (default 0.01)" << std::endl;
	std::cout << "     --scene_ss val:         Scene uniform sampling radius (default 0.03)" << std::endl;
	std::cout << "     --rf_rad val:           Reference frame radius (default 0.015)" << std::endl;
	std::cout << "     --descr_rad val:        Descriptor radius (default 0.02)" << std::endl;
	std::cout << "     --cg_size val:          Cluster size (default 0.01)" << std::endl;
	std::cout << "     --cg_thresh val:        Clustering threshold (default 5)" << std::endl;
	std::cout << "     --downsampling val: enter the type of downsampling" << std::endl << std::endl;
}

void
parseCommandLine(int argc, char *argv[])
{
	//Show help
	if (pcl::console::find_switch(argc, argv, "-h"))
	{
		showHelp(argv[0]);
		exit(0);
	}

	//Model & scene filenames
	std::vector<int> filenames;
	filenames = pcl::console::parse_file_extension_argument(argc, argv, ".pcd");
	if (filenames.size() != 2)
	{
		std::cout << "Filenames missing.\n";
		showHelp(argv[0]);
		exit(-1);
	}

	model_filename_ = argv[filenames[0]];
	scene_filename_ = argv[filenames[1]];

	//Program behavior
	if (pcl::console::find_switch(argc, argv, "-k"))
	{
		show_keypoints_ = true;
	}
	if (pcl::console::find_switch(argc, argv, "-c"))
	{
		show_correspondences_ = true;
	}
	if (pcl::console::find_switch(argc, argv, "-r"))
	{
		use_cloud_resolution_ = true;
	}

	std::string used_sampling_algorithm;
	if (pcl::console::parse_argument(argc, argv, "--algorithm", used_sampling_algorithm) != -1)
	{
		if (used_sampling_algorithm.compare("vg") == 0)
		{
			use_vg_ = true;
		}
		else if (used_sampling_algorithm.compare("us") == 0)
		{
			use_vg_ = false;
		}
		else
		{
			std::cout << "Wrong algorithm name.\n";
			showHelp(argv[0]);
			exit(-1);
		}
	}

	std::string used_algorithm;
	if (pcl::console::parse_argument(argc, argv, "--algorithm", used_algorithm) != -1)
	{
		if (used_algorithm.compare("Hough") == 0)
		{
			use_hough_ = true;
		}
		else if (used_algorithm.compare("GC") == 0)
		{
			use_hough_ = false;
		}
		else
		{
			std::cout << "Wrong algorithm name.\n";
			showHelp(argv[0]);
			exit(-1);
		}
	}

	//General parameters
	pcl::console::parse_argument(argc, argv, "--model_ss", model_ss_);
	pcl::console::parse_argument(argc, argv, "--scene_ss", scene_ss_);
	pcl::console::parse_argument(argc, argv, "--rf_rad", rf_rad_);
	pcl::console::parse_argument(argc, argv, "--descr_rad", descr_rad_);
	pcl::console::parse_argument(argc, argv, "--cg_size", cg_size_);
	pcl::console::parse_argument(argc, argv, "--cg_thresh", cg_thresh_);
}

double
computeCloudResolution(const pcl::PointCloud<PointType>::ConstPtr &cloud)
{
	double res = 0.0;
	int n_points = 0;
	int nres;
	std::vector<int> indices(2);
	std::vector<float> sqr_distances(2);
	pcl::search::KdTree<PointType> tree;
	tree.setInputCloud(cloud);

	for (size_t i = 0; i < cloud->size(); ++i)
	{
		if (!pcl_isfinite((*cloud)[i].x))
		{
			continue;
		}
		//Considering the second neighbor since the first is the point itself.
		nres = tree.nearestKSearch(i, 2, indices, sqr_distances);
		if (nres == 2)
		{
			res += sqrt(sqr_distances[1]);
			++n_points;
		}
	}
	if (n_points != 0)
	{
		res /= n_points;
	}
	return res;
}

int
main(int argc, char *argv[])
{
	parseCommandLine(argc, argv);
	pcl::PointCloud<PointType>::Ptr model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr model_keypoints(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr scene_keypoints(new pcl::PointCloud<PointType>());
	pcl::PointCloud<NormalType>::Ptr model_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<NormalType>::Ptr scene_normals(new pcl::PointCloud<NormalType>());
	pcl::PointCloud<DescriptorType>::Ptr model_descriptors(new pcl::PointCloud<DescriptorType>());
	pcl::PointCloud<DescriptorType>::Ptr scene_descriptors(new pcl::PointCloud<DescriptorType>());

	//
	//  Load clouds
	//
	if (pcl::io::loadPCDFile(model_filename_, *model) < 0)
	{
		std::cout << "Error loading model cloud." << std::endl;
		showHelp(argv[0]);
		return (-1);
	}
	if (pcl::io::loadPCDFile(scene_filename_, *scene) < 0)
	{
		std::cout << "Error loading scene cloud." << std::endl;
		showHelp(argv[0]);
		return (-1);
	}

	//
	//  Set up resolution invariance
	//
	if (use_cloud_resolution_)
	{
		float resolution = static_cast<float> (computeCloudResolution(model));
		if (resolution != 0.0f)
		{
			model_ss_ *= resolution;
			scene_ss_ *= resolution;
			rf_rad_ *= resolution;
			descr_rad_ *= resolution;
			cg_size_ *= resolution;
		}

		std::cout << "Model resolution:       " << resolution << std::endl;
		std::cout << "Model sampling size:    " << model_ss_ << std::endl;
		std::cout << "Scene sampling size:    " << scene_ss_ << std::endl;
		std::cout << "LRF support radius:     " << rf_rad_ << std::endl;
		std::cout << "SHOT descriptor radius: " << descr_rad_ << std::endl;
		std::cout << "Clustering bin size:    " << cg_size_ << std::endl << std::endl;
	}
	cout << "Number of points in model: " << model->points.size() << endl;
	cout << "Number of points in scene: " << scene->points.size() << endl;
	clock_t begin = clock();
	clock_t begin_scratch = clock();
	//
	//  Compute Normals
	//
	pcl::NormalEstimation<PointType, NormalType> norm_est;
	norm_est.setKSearch(11);
	norm_est.setInputCloud(model);
	norm_est.compute(*model_normals);

	norm_est.setInputCloud(scene);
	norm_est.compute(*scene_normals);

	clock_t compute_normals = clock();
	double elapsed_secs1 = double(compute_normals - begin) / CLOCKS_PER_SEC;


	//
	//  Downsample Clouds to Extract keypoints
	//
	begin = clock();
	if (use_vg_ == true){
		
		

	}
	else
	{
		pcl::UniformSampling<PointType> uniform_sampling;
		uniform_sampling.setInputCloud(model);
		uniform_sampling.setRadiusSearch(model_ss_);
		uniform_sampling.filter(*model_keypoints);
		std::cout << "Model total points: " << model->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;

		uniform_sampling.setInputCloud(scene);
		uniform_sampling.setRadiusSearch(scene_ss_);
		uniform_sampling.filter(*scene_keypoints);
		std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;
	}
	clock_t un_sampling = clock();
	double elapsed_secs2 = double(un_sampling - begin) / CLOCKS_PER_SEC;
	//
	//  Compute Descriptor for keypoints
	//
	begin = clock();
	pcl::SHOTEstimation<PointType, NormalType, DescriptorType> descr_est;
	descr_est.setRadiusSearch(descr_rad_);

	descr_est.setInputCloud(model_keypoints);
	descr_est.setInputNormals(model_normals);
	descr_est.setSearchSurface(model);
	descr_est.compute(*model_descriptors);

	descr_est.setInputCloud(scene_keypoints);
	descr_est.setInputNormals(scene_normals);
	descr_est.setSearchSurface(scene);
	descr_est.compute(*scene_descriptors);
	

	


	/*pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308> descr_est1;
	descr_est1.setInputCloud(model_keypoints);
	descr_est1.setInputNormals(model_normals);
	//descr_est1.setSearchMethod(kdtree);
	// Optionally, we can normalize the bins of the resulting histogram,
	// using the total number of points.
	descr_est1.setNormalizeBins(true);
	// Also, we can normalize the SDC with the maximum size found between
	// the centroid and any of the cluster's points.
	descr_est1.setNormalizeDistance(false);
	pcl::PointCloud<pcl::VFHSignature308>::Ptr model_descriptors1(new pcl::PointCloud<pcl::VFHSignature308>());
	pcl::PointCloud<pcl::VFHSignature308>::Ptr scene_descriptors1(new pcl::PointCloud<pcl::VFHSignature308>());
	descr_est1.compute(*model_descriptors1);

	descr_est1.setInputCloud(scene_keypoints);
	descr_est1.setInputNormals(scene_normals);
	descr_est1.setSearchSurface(scene);
	descr_est1.compute(*scene_descriptors1);
	*/

	/*pcl::UniqueShapeContext<pcl::PointXYZ, pcl::UniqueShapeContext1960, pcl::ReferenceFrame> descr_est1;
	descr_est1.setInputCloud(model_keypoints);
	descr_est1.setInputNormals(model_normals);
	//descr_est1.setSearchMethod(kdtree);
	// Optionally, we can normalize the bins of the resulting histogram,
	// using the total number of points.
	descr_est1.setNormalizeBins(true);
	// Also, we can normalize the SDC with the maximum size found between
	// the centroid and any of the cluster's points.
	descr_est1.setNormalizeDistance(false);
	pcl::PointCloud<pcl::VFHSignature308>::Ptr model_descriptors1(new pcl::PointCloud<pcl::VFHSignature308>());
	pcl::PointCloud<pcl::VFHSignature308>::Ptr scene_descriptors1(new pcl::PointCloud<pcl::VFHSignature308>());
	descr_est1.compute(*model_descriptors1);

	descr_est1.setInputCloud(scene_keypoints);
	descr_est1.setInputNormals(scene_normals);
	descr_est1.setSearchSurface(scene);
	descr_est1.compute(*scene_descriptors1);
	*/

	/*pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr desc1(new pcl::PointCloud<pcl::UniqueShapeContext1960>());

	// Note: you would usually perform downsampling now. It has been omitted here
	// for simplicity, but be aware that computation can take a long time.

	// USC estimation object.
	pcl::UniqueShapeContext<pcl::PointXYZ, pcl::UniqueShapeContext1960, pcl::ReferenceFrame> usc;
	usc.setInputCloud(model_keypoints);
	// Search radius, to look for neighbors. It will also be the radius of the support sphere.
	usc.setRadiusSearch(0.05);
	// The minimal radius value for the search sphere, to avoid being too sensitive
	// in bins close to the center of the sphere.
	usc.setMinimalRadius(0.05 / 10.0);
	// Radius used to compute the local point density for the neighbors
	// (the density is the number of points within that radius).
	usc.setPointDensityRadius(0.05 / 5.0);
	// Set the radius to compute the Local Reference Frame.
	usc.setLocalRadius(0.05);

	usc.compute(*desc1);


	pcl::PointCloud<pcl::UniqueShapeContext1960>::Ptr desc2(new pcl::PointCloud<pcl::UniqueShapeContext1960>());

	// Note: you would usually perform downsampling now. It has been omitted here
	// for simplicity, but be aware that computation can take a long time.

	// USC estimation object.
	//pcl::UniqueShapeContext<pcl::PointXYZ, pcl::UniqueShapeContext1960, pcl::ReferenceFrame> usc;
	usc.setInputCloud(scene_keypoints);
	// Search radius, to look for neighbors. It will also be the radius of the support sphere.
	usc.setRadiusSearch(0.05);
	// The minimal radius value for the search sphere, to avoid being too sensitive
	// in bins close to the center of the sphere.
	usc.setMinimalRadius(0.05 / 10.0);
	// Radius used to compute the local point density for the neighbors
	// (the density is the number of points within that radius).
	usc.setPointDensityRadius(0.05 / 5.0);
	// Set the radius to compute the Local Reference Frame.
	usc.setLocalRadius(0.05);

	usc.compute(*desc2);
	*/
	clock_t compute_descriptors = clock();
	double elapsed_secs3 = double(compute_descriptors - begin) / CLOCKS_PER_SEC;

	//
	//  Find Model-Scene Correspondences with KdTree
	//
	begin = clock();
	pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

	pcl::KdTreeFLANN<DescriptorType> match_search;
	match_search.setInputCloud(desc1);

	//  For each scene keypoint descriptor, find nearest neighbor into the model keypoints descriptor cloud and add it to the correspondences vector.
	for (size_t i = 0; i < desc2->size(); ++i)
	{
		std::vector<int> neigh_indices(1);
		std::vector<float> neigh_sqr_dists(1);
		if (!pcl_isfinite(scene_descriptors->at(i).descriptor[0])) //skipping NaNs
		{
			continue;
		}
		int found_neighs = match_search.nearestKSearch(desc2->at(i), 1, neigh_indices, neigh_sqr_dists);
		//cout << neigh_sqr_dists[0] << endl;
		if (found_neighs == 1 && neigh_sqr_dists[0] < 30000.0f) //  add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
		{//
			pcl::Correspondence corr(neigh_indices[0], static_cast<int> (i), neigh_sqr_dists[0]);
			model_scene_corrs->push_back(corr);
		}
	}
	std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;
	clock_t end = clock();
	double elapsed_secs4 = double(end - begin) / CLOCKS_PER_SEC;
	//
	//  Actual Clustering
	//
	std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f> > rototranslations;
	std::vector<pcl::Correspondences> clustered_corrs;
	begin = clock();
	//  Using Hough3D
	if (use_hough_)
	{
		//
		//  Compute (Keypoints) Reference Frames only for Hough
		//
		pcl::PointCloud<RFType>::Ptr model_rf(new pcl::PointCloud<RFType>());
		pcl::PointCloud<RFType>::Ptr scene_rf(new pcl::PointCloud<RFType>());

		pcl::BOARDLocalReferenceFrameEstimation<PointType, NormalType, RFType> rf_est;
		rf_est.setFindHoles(true);
		rf_est.setRadiusSearch(rf_rad_);
		rf_est.setInputCloud(model_keypoints);
		rf_est.setInputNormals(model_normals);
		rf_est.setSearchSurface(model);
		rf_est.compute(*model_rf);
		rf_est.setInputCloud(scene_keypoints);
		rf_est.setInputNormals(scene_normals);
		rf_est.setSearchSurface(scene);
		rf_est.compute(*scene_rf);

		//  Clustering
		pcl::Hough3DGrouping<PointType, PointType, RFType, RFType> clusterer;
		clusterer.setHoughBinSize(cg_size_);
		clusterer.setHoughThreshold(cg_thresh_);
		clusterer.setUseInterpolation(true);
		clusterer.setUseDistanceWeight(false);
		clusterer.setInputCloud(model_keypoints);
		clusterer.setInputRf(model_rf);
		clusterer.setSceneCloud(scene_keypoints);
		clusterer.setSceneRf(scene_rf);
		clusterer.setModelSceneCorrespondences(model_scene_corrs);
		clusterer.recognize(rototranslations, clustered_corrs);
	}
	else // Using GeometricConsistency
	{ /*
		pcl::GeometricConsistencyGrouping<PointType, PointType> gc_clusterer;
		gc_clusterer.setGCSize(cg_size_);
		gc_clusterer.setGCThreshold(cg_thresh_);

		gc_clusterer.setInputCloud(model_keypoints);
		gc_clusterer.setSceneCloud(scene_keypoints);
		gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

		//gc_clusterer.cluster (clustered_corrs);
		gc_clusterer.recognize(rototranslations, clustered_corrs);
		*/
	}
	end = clock();
	double elapsed_secs5 = double(end - begin) / CLOCKS_PER_SEC;
	cout << "Elapsed Time for Hough Grouping " << elapsed_secs5 << endl;
	//
	//  Output results
	//
	std::cout << "Model instances found: " << rototranslations.size() << std::endl;
	for (size_t i = 0; i < rototranslations.size(); ++i)
	{
		std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
		std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;
		// Print the rotation matrix and translation vector
		Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);
		Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);
		printf("\n");
		printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
		printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
		printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
		printf("\n");
		printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
	}
	//
	//  Visualization
	//
	pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
	viewer.addPointCloud(scene, "scene_cloud");

	pcl::PointCloud<PointType>::Ptr off_scene_model(new pcl::PointCloud<PointType>());
	pcl::PointCloud<PointType>::Ptr off_scene_model_keypoints(new pcl::PointCloud<PointType>());
	if (show_correspondences_ || show_keypoints_)
	{
		//  We are translating the model so that it doesn't end in the middle of the scene representation
		pcl::transformPointCloud(*model, *off_scene_model, Eigen::Vector3f(-3, 0, 0), Eigen::Quaternionf(5, 0, 0, 0));
		pcl::transformPointCloud(*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f(-3, 0, 0), Eigen::Quaternionf(5, 0, 0, 0));

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_color_handler(off_scene_model, 255, 255, 128);
		viewer.addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");

	}
	if (show_keypoints_)
	{
		pcl::visualization::PointCloudColorHandlerCustom<PointType> scene_keypoints_color_handler(scene_keypoints, 0, 0, 255);
		viewer.addPointCloud(scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

		pcl::visualization::PointCloudColorHandlerCustom<PointType> off_scene_model_keypoints_color_handler(off_scene_model_keypoints, 0, 0, 255);
		viewer.addPointCloud(off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
		viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
	}
	for (size_t i = 0; i < rototranslations.size(); ++i)
	{
		pcl::PointCloud<PointType>::Ptr rotated_model(new pcl::PointCloud<PointType>());
		pcl::transformPointCloud(*model, *rotated_model, rototranslations[i]);

		std::stringstream ss_cloud;
		ss_cloud << "instance" << i;

		pcl::visualization::PointCloudColorHandlerCustom<PointType> rotated_model_color_handler(rotated_model, 255, 0, 0);
		viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());
		if (show_correspondences_)
		{
			for (size_t j = 0; j < clustered_corrs[i].size(); ++j)
			{
				std::stringstream ss_line;
				ss_line << "correspondence_line" << i << "_" << j;
				PointType& model_point = off_scene_model_keypoints->at(clustered_corrs[i][j].index_query);
				PointType& scene_point = scene_keypoints->at(clustered_corrs[i][j].index_match);

				//  We are drawing a line for each pair of clustered correspondences found between the model and the scene
				viewer.addLine<PointType, PointType>(model_point, scene_point, 0, 255, 0, ss_line.str());
			}
		}
	}
	
     end = clock();
	double elapsed_secs = double(end - begin_scratch) / CLOCKS_PER_SEC;
	cout << "Elapsed Time Overall: " << elapsed_secs << endl;
	cout << "Elapsed Time for Normal computation: " << elapsed_secs1 << endl;
	cout << "Elapsed Time for Uniform sampling " << elapsed_secs2 << endl;
	cout << "Elapsed Time for Computing Descriptors " << elapsed_secs3 << endl;
	cout << "Elapsed Time for finding correspondences " << elapsed_secs4 << endl;
	cout << "Elapsed Time for Hough Grouping " << elapsed_secs5 << endl;
	cout << "Number of points in model: " << model->points.size() << endl;
	cout << "Number of points in scene: " << scene->points.size() << endl;
	while (!viewer.wasStopped())
	{
		viewer.spinOnce();
	}

	return (0);
}
