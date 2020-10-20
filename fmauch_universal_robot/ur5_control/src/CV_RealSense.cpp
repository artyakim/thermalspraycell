//
// Created by artemyakimchuk on 17/12/2019.
//

#include <ros/ros.h>
 // PCL specific includes

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
 #include <pcl/filters/voxel_grid.h>

#include <iostream>
#include <string>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <Eigen/Core>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/common/time.h>
#include <pcl/console/print.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/io/io.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/io.h>
#include <tf/transform_broadcaster.h>
#include "librealsense2/rs.hpp"

typedef pcl::PointXYZ PointT;

//сделать глобальную переменную для входящего поинт клауда
//передать его в мейн
//входящий поинт клауд присвоить к cloud_icp
//посмотреть на вывод
ros::Publisher pub;

typedef pcl::PointNormal PointNT;
typedef pcl::PointCloud<PointNT> PointCloudT;
typedef pcl::FPFHSignature33 FeatureT;
typedef pcl::FPFHEstimationOMP<PointNT,PointNT,FeatureT> FeatureEstimationT;
typedef pcl::PointCloud<FeatureT> FeatureCloudT;
typedef pcl::visualization::PointCloudColorHandlerCustom<PointNT> ColorHandlerT;
typedef pcl::PointCloud<PointT> PointCloudTI;


void print4x4Matrix (const Eigen::Matrix4d & matrix)
{
	printf ("Rotation matrix :\n");
	printf ("    | %6.3f %6.3f %6.3f | \n", matrix (0, 0), matrix (0, 1), matrix (0, 2));
	printf ("R = | %6.3f %6.3f %6.3f | \n", matrix (1, 0), matrix (1, 1), matrix (1, 2));
	printf ("    | %6.3f %6.3f %6.3f | \n", matrix (2, 0), matrix (2, 1), matrix (2, 2));
	printf ("Translation vector :\n");
	printf ("t = < %6.3f, %6.3f, %6.3f >\n\n", matrix (0, 3), matrix (1, 3), matrix (2, 3));
}

Eigen::Matrix4d PCAlignment(PointCloudT::Ptr scene_rviz, PointCloudT::Ptr object)
{
	Eigen::Matrix4d error;
	PointCloudT::Ptr object_aligned (new PointCloudT);
	PointCloudT::Ptr scene (new PointCloudT);
	FeatureCloudT::Ptr object_features (new FeatureCloudT);
	FeatureCloudT::Ptr scene_features (new FeatureCloudT);

	// Define min and max for X, Y and Z
	float minX = -0.25, minY = -0.15, minZ = -0.5;
	float maxX = +0.25, maxY = +0.15, maxZ = +0.6;

	pcl::CropBox<PointNT> boxFilter;
	boxFilter.setMin(Eigen::Vector4f(minX, minY, minZ, 1.0));
	boxFilter.setMax(Eigen::Vector4f(maxX, maxY, maxZ, 1.0));
	boxFilter.setInputCloud(scene_rviz);
	boxFilter.filter(*scene);

	// Downsample
	pcl::console::print_highlight ("Downsampling...\n");
	pcl::VoxelGrid<PointNT> grid;
	const float leaf = 0.005f;
	grid.setLeafSize (leaf, leaf, leaf);
	grid.setInputCloud (object);
	grid.filter (*object);
	const float leaf1 = 0.005f;
	grid.setLeafSize (leaf1, leaf1, leaf1);
	grid.setInputCloud (scene);
	grid.filter (*scene);


	// Estimate normals for scene
	pcl::console::print_highlight ("Estimating scene normals...\n");
	pcl::NormalEstimation<PointNT,PointNT> nest;
	nest.setRadiusSearch (0.01);
	nest.setInputCloud (scene);
	nest.compute (*scene);
//	 nest.setInputCloud (object);
//	 nest.compute (*object);

	// Estimate features
	pcl::console::print_highlight ("Estimating features...\n");
	FeatureEstimationT fest;
	fest.setRadiusSearch (0.025);
	fest.setInputCloud (object);
	fest.setInputNormals (object);
	fest.compute (*object_features);
	fest.setInputCloud (scene);
	fest.setInputNormals (scene);
	fest.compute (*scene_features);

	// Perform alignment
	pcl::console::print_highlight ("Starting alignment...\n");
	pcl::SampleConsensusPrerejective<PointNT,PointNT,FeatureT> align;
	Eigen::Matrix4d transformation;

	align.setInputSource (object);
	align.setSourceFeatures (object_features);
	align.setInputTarget (scene);
	align.setTargetFeatures (scene_features);
	align.setMaximumIterations (300000); // Number of RANSAC iterations
	align.setNumberOfSamples (3); // Number of points to sample for generating/prerejecting a pose
	align.setCorrespondenceRandomness (20); // Number of nearest features to use
	align.setSimilarityThreshold (0.9f);// Polygonal edge length similarity threshold
	align.setMaxCorrespondenceDistance (1.5f * leaf); // Inlier threshold
	align.setInlierFraction (0.3); // Required inlier fraction for accepting a pose hypothesis
	{
		pcl::ScopeTime t("Alignment");
		align.align (*object_aligned);
	}

	if (align.hasConverged ())
	{
		// Print results
		printf ("\n");
		transformation = align.getFinalTransformation().cast<double>();
		pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (0,0), transformation (0,1), transformation (0,2));
		pcl::console::print_info ("R = | %6.3f %6.3f %6.3f | \n", transformation (1,0), transformation (1,1), transformation (1,2));
		pcl::console::print_info ("    | %6.3f %6.3f %6.3f | \n", transformation (2,0), transformation (2,1), transformation (2,2));
		pcl::console::print_info ("\n");
		pcl::console::print_info ("t = < %0.3f, %0.3f, %0.3f >\n", transformation (0,3), transformation (1,3), transformation (2,3));
		pcl::console::print_info ("\n");
		pcl::console::print_info ("Inliers: %i/%i\n", align.getInliers ().size (), object->size ());
		pcl::console::print_info ("Inliers Fraction: %i/%i\n", align.getInlierFraction(), object->size ());

		 //Show alignment
		 pcl::visualization::PCLVisualizer visu("Alignment");
		 visu.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 255.0), "scene");
		 visu.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 255.0, 255.0, 0.0), "object_aligned");
		 visu.spin ();
	}
	else
	{
		pcl::console::print_error ("Alignment RANSAC failed!\n");
		return error.setZero();
	}


	Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity ();

	pcl::IterativeClosestPoint<PointNT, PointNT> icp;
	icp.setMaximumIterations (150);
	icp.setTransformationEpsilon(1e-9);
	icp.setMaxCorrespondenceDistance(0.005f);
	icp.setInputSource (object_aligned);
	icp.setInputTarget (scene);
	icp.align (*object_aligned);

	if (icp.hasConverged ())
	{
		std::cout << "\nICP has converged, score is " << icp.getFitnessScore () << std::endl;
		std::cout << "\nICP transformation " << 5 << " : cloud_icp -> cloud_in" << std::endl;
		transformation_matrix = icp.getFinalTransformation ().cast<double>();
		print4x4Matrix (transformation_matrix);
		// Show alignment
		pcl::visualization::PCLVisualizer visu2("Alignment");
		visu2.addPointCloud (scene, ColorHandlerT (scene, 0.0, 255.0, 255.0), "scene");
		visu2.addPointCloud (object_aligned, ColorHandlerT (object_aligned, 255.0, 255.0, 0.0), "object_aligned");
		visu2.spin ();
	}
	else
	{
		PCL_ERROR ("\nICP has not converged.\n");
		return error.setZero();
	}

	return  (transformation_matrix*transformation);
}

PointCloudT::Ptr cloud_from_sensor (new PointCloudT);

tf::StampedTransform FromEigenToStpTrans(Eigen::Matrix4d to_convert)
{
	tf::StampedTransform obj_trans;
	tf::Matrix3x3 rot_obj (
			to_convert(0,0),to_convert(0,1),to_convert(0,2),
			to_convert(1,0),to_convert(1,1),to_convert(1,2),
			to_convert(2,0),to_convert(2,1),to_convert(2,2)
	);
	obj_trans.getOrigin().setX(to_convert(0,3));
	obj_trans.getOrigin().setY(to_convert(1,3));
	obj_trans.getOrigin().setZ(to_convert(2,3));
	obj_trans.setBasis(rot_obj);

	return obj_trans;

}

 void cloud_get (const sensor_msgs::PointCloud2ConstPtr &cloud_raw)
 {
	 pcl::PCLPointCloud2 cloud;
	 pcl::fromROSMsg (*cloud_raw, *cloud_from_sensor);
	 pcl::console::print_highlight ("CALLBACK...\n");
 }


 int

 main (int argc,
 		char** argv)
 {
   // Initialize ROS
   ros::init (argc, argv, "my_pcl_tutorial");
   ros::NodeHandle nh;
   static tf::TransformBroadcaster br;

   // Create a ROS publisher for the output point cloud
   //pub = nh.advertise<pcl::PCLPointCloud2> ("output", 1);
   ros::Subscriber sub = nh.subscribe<sensor_msgs::PointCloud2>("/camera/depth/color/points",10, cloud_get);
	 PointCloudT::Ptr scene_rviz (new PointCloudT);


// Point clouds
	 PointCloudT::Ptr object (new PointCloudT);
	 PointCloudT::Ptr object1 (new PointCloudT);
	 PointCloudT::Ptr object_aligned (new PointCloudT);
	 PointCloudT::Ptr scene (new PointCloudT);
	 PointCloudT::Ptr scene1 (new PointCloudT);
	 FeatureCloudT::Ptr object_features (new FeatureCloudT);
	 FeatureCloudT::Ptr scene_features (new FeatureCloudT);

	 // Get input object and scene
	 if (argc != 3)
	 {
		 pcl::console::print_error ("Syntax is: %s object.pcd scene.pcd\n", argv[0]);
		 return (1);
	 }
	 // Load object and scene
	 pcl::console::print_highlight ("Loading point clouds...\n");
	 if (pcl::io::loadPLYFile<PointNT> (argv[1], *object) < 0 ||
		 pcl::io::loadPLYFile<PointNT> (argv[2], *scene1) < 0)
	 {
		 pcl::console::print_error ("Error loading object/scene file!\n");
		 return (1);
	 }


	 ros::Rate loop_rate(10);
	 while(ros::ok()) {
		 while (cloud_from_sensor->empty()) {
			 pcl::console::print_highlight("NO CLOUD HERE...\n");
			 ros::spinOnce();
		 }
		 scene_rviz = cloud_from_sensor;
//////////////////////////////////////////////////////////////////////////////////////PC Processing
		 Eigen::Matrix4d final_tf = PCAlignment(scene_rviz, object);
//////////////////////////////////////////////////////////////////////////////////////PC Processing -final
		 tf::StampedTransform obj_trans = FromEigenToStpTrans(final_tf);//position conversion
		 obj_trans.stamp_ = ros::Time(0);
		 obj_trans.frame_id_ = "camera_depth_optical_frame";
		 obj_trans.child_frame_id_ = "object_pose";
		 br.sendTransform(obj_trans);
		 // Spin
		 //ros::spin();
		 ros::spinOnce();
		 loop_rate.sleep();
	 }
 }