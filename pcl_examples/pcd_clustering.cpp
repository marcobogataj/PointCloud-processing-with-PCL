//PCD CLUSTERING TESTING

//utility libraries
#include <iostream>
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

//specific processing libraries
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>

typedef pcl::PointXYZRGBA PointT;

int main()
{
  // Load PCD file
  pcl::PointCloud<PointT>::Ptr raw_cloud (new pcl::PointCloud<PointT>);

  std::string path = "/Users/marcobogataj/Documents/UNI/magistrale/BG/THESIS/Tesi ABB/zivid_captures/";
  if (pcl::io::loadPCDFile<PointT> (path+std::string("zivid_manual_holefilling.pcd"), *raw_cloud) == -1) //* load the file
  {
      PCL_ERROR ("Couldn't read the .PLY file\n");
      return (-1);
  }

  std::cout<<"Source Cloud Points "<< raw_cloud->width * raw_cloud->height<< std::endl;





  // START of Filtering (1-Voxelization,2-Passthrough,3-Outliers ...)
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>); //filtered cloud 

  // 1-VOXELIZE
  pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setInputCloud(raw_cloud);
  voxel_filter.setLeafSize(1,1,1);
  voxel_filter.filter(*cloud);
  *raw_cloud = *cloud; //save to raw cloud for a new filter

  // 2-PASSTHROUGH
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (raw_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 500.0);
  //pass.setNegative (true);
  pass.filter (*cloud);
  *raw_cloud = *cloud; //save to raw cloud for a new filter

  // 3-STATISTICAL OUTLIER
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud (raw_cloud);
  sor.setMeanK (100);
  sor.setStddevMulThresh (1.5);
  sor.filter (*cloud);

  std::cout<<"Filtered Cloud Points "<< cloud->width * cloud->height<< std::endl;
  // END of Filtering
  
  



  // START of Transform (better visualisation)
  //Transformation using a Affine3f for BETTER VISUALIZATION of the PointCloud
  Eigen::Affine3f visual_transform = Eigen::Affine3f::Identity();

  // Define a translation of 2.5 meters on the x axis.
  visual_transform.translation() << 0.0, 0.0, 400.0;
  // Define a 180° of rotation around X axis
  float theta = M_PI; 
  visual_transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));

  // Executing the transformation
  pcl::PointCloud<PointT>::Ptr t_cloud (new pcl::PointCloud<PointT>);
  pcl::transformPointCloud(*cloud, *t_cloud, visual_transform);

  *cloud = *t_cloud;  // Copy t_cloud to cloud to be processed and keep t_cloud as starting reference
  // END of Transform 
  




  // START OF SEGMENTATION (1-Planar, 2-Cylindrical{TO DO!!} , ...)

  // 1-PLANAR SEGMENTATION
  // Declare segmentation pointers
  pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);

  //Declare point clouds for segmentation
  pcl::PointCloud<PointT>::Ptr pln_cloud (new pcl::PointCloud<PointT>);
  
  //Declare temporary clouds used for the iterative segmentation
  pcl::PointCloud<PointT>::Ptr obj_cloud_temp (new pcl::PointCloud<PointT>), pln_cloud_temp (new pcl::PointCloud<PointT>);

  int original_size(cloud->height*cloud->width);
  int n_planes(0);
  
  // Set segmantion options:
  Eigen::Vector3f plane_axis(0,0,1); //find planes perpendicular to x axis
  double theta_eps = M_PI/5;
  float _min_percentage = 0.2; //size threshold of the point cloud for stopping planar segmentation
  float _max_distance = 1; //size threshold of the point cloud for stopping planar segmentation

  pcl::SACSegmentation<PointT> pln_seg;
  pln_seg.setModelType(pcl::SACMODEL_PLANE);
  pln_seg.setMethodType(pcl::SAC_RANSAC);
  pln_seg.setDistanceThreshold(_max_distance);
  pln_seg.setOptimizeCoefficients(true);
  pln_seg.setMaxIterations(100);
  //pln_seg.setEpsAngle(theta_eps);
  //pln_seg.setAxis(plane_axis);

  // Declare extraction pointer
  pcl::ExtractIndices<PointT> extract; 

  //Declare vector of pointers containing all the segmented planes
  //std::vector <pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator <pcl::PointCloud<PointT>::Ptr>> v_pln_cloud;

  // Planar clustering loop
  printf ("Start planar clustering...\n");
  int nr_points = (int) cloud->size (); //compute size of initial point cloud to exit while()
  while (cloud->size() > _min_percentage * nr_points){

        // Fit a plane
        pln_seg.setInputCloud(cloud);
        pln_seg.segment(*inliers, *coefficients);

        // Check result
        if (inliers->indices.size () == 0)
        {
          std::cout << "Could not estimate a planar model for plane n°"<< n_planes+1 << std::endl << std::endl;
          break;
        }
        
        // Extract planes inliers 
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(false);
        extract.filter(*pln_cloud_temp);

        //concatenate plane points;
        *pln_cloud += *pln_cloud_temp;

        // Add plane to vector of pointers
        //v_pln_cloud.push_back(pln_cloud_temp);

        // Extract objects inliers 
        extract.setInputCloud(cloud);
        extract.setIndices(inliers);
        extract.setNegative(true);
        extract.filter(*obj_cloud_temp);
        *cloud = *obj_cloud_temp;

        // Display info
        std::cerr << "Fitted plane: " << coefficients->values[0] << "x + " 
                                      << coefficients->values[1] << "y + "
                                      << coefficients->values[2] << "z + " 
                                      << coefficients->values[3] << "= 0"<<std::endl;

        std::cout<< "Points left in cloud:" << cloud->width*cloud->height <<std::endl;

        // Nest iteration
        n_planes++;
    }
  printf ("Planar clustering iteraion finished.\n");
  std::cout<< "Number of planes segmented: " << n_planes <<std::endl<<std::endl;



  // 2-CYLINDRICAL SEGMENTATION
  // {TO DO!!!}

  // END of SEGMENTATION









  // Visualization using PCLVisualizer
  pcl::visualization::PCLVisualizer viewer ("Cloud viewer");
  
  int v1(0);
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer.setBackgroundColor (0, 0, 0, v1);
  viewer.addText ("Source point cloud",10,10,"v1 text", v1);
  viewer.addPointCloud<PointT> (t_cloud, "t_cloud", v1);

  int v2(0);
  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
  viewer.setBackgroundColor (0.3, 0.3, 0.3, v2);
  viewer.addText ("Segmented point cloud", 10, 10, "v2 text", v2);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> obj_color (cloud, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> pln_color (pln_cloud, 0, 255, 0);
  
  //add objects point cloud (shown in red)
  viewer.addPointCloud<PointT> (cloud, "obj_cloud", v2);
  
  //add extracted plane point cloud (shown in green)
  viewer.addPointCloud<PointT> (pln_cloud, pln_color, "pln_clouds", v2);

  //add sphere in (0,0,0) with radius 500 for debugging filters
  /*
  pcl::PointXYZ C(0,  0, 0);
  viewer.addSphere (C, 500, 0, 0, 0.5, "sphere",v2);
  */

  //visual utilities
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "t_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "obj_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "pln_clouds");
  viewer.addCoordinateSystem (100);
  viewer.setBackgroundColor(255, 255, 255); // Setting background color

  while (!viewer.wasStopped ()) { // Display the visualiser 
  viewer.spinOnce ();
  }
  return (0);
}