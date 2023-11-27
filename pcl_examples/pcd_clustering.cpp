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
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/features/normal_3d.h>

typedef pcl::PointXYZRGBA PointT;

int main()
{
  // Load PCD file
  pcl::PointCloud<PointT>::Ptr raw_cloud (new pcl::PointCloud<PointT>);

  std::string path = "/Users/marcobogataj/Documents/UNI/magistrale/BG/THESIS/Tesi ABB/zivid_captures/";
  if (pcl::io::loadPCDFile<PointT> (path+std::string("zivid_manual_holefilling.pcd"), *raw_cloud) == -1) //* load the file
  {
      PCL_ERROR ("Couldn't read the .PCD file\n");
      return (-1);
  }

  std::cout<<"Source Cloud Points "<< raw_cloud->width * raw_cloud->height<< std::endl;


  // START of Transform (better visualisation)
  //Transformation using a Affine3f for BETTER VISUALIZATION of the PointCloud
  Eigen::Affine3f visual_transform = Eigen::Affine3f::Identity();

  // Define a translation of 2.5 meters on the x axis.
  visual_transform.translation() << 0.0, 0.0, 400.0;
  // Define a 180° of rotation around X axis
  float theta = M_PI; 
  visual_transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitX()));

  // Executing the transformation
  pcl::PointCloud<PointT>::Ptr t_raw_cloud (new pcl::PointCloud<PointT>);
  pcl::transformPointCloud(*raw_cloud, *t_raw_cloud, visual_transform);

  *raw_cloud = *t_raw_cloud;  // Copy t_raw_cloud to raw_cloud to be processed and keep t_cloud as starting reference
  // END of Transform 


  // START of Filtering (1-Voxelization,2-Passthrough,3-Outliers ...)
  pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>); //filtered cloud 

  // 1-VOXELIZE
  pcl::VoxelGrid<PointT> voxel_filter;
  voxel_filter.setInputCloud(raw_cloud);
  voxel_filter.setLeafSize(0.4f,0.4f,0.4f);
  voxel_filter.filter(*cloud);
  *raw_cloud = *cloud; //save to raw cloud for a new filter

  // 2-PASSTHROUGH
  pcl::PassThrough<PointT> pass;
  pass.setInputCloud (raw_cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (-100, 100);
  //pass.setNegative (true);
  pass.filter (*cloud);
  *raw_cloud = *cloud; //save to raw cloud for a new filter

  // 3-STATISTICAL OUTLIER
  pcl::StatisticalOutlierRemoval<PointT> sor;
  sor.setInputCloud (raw_cloud);
  sor.setMeanK (100);
  sor.setStddevMulThresh (1);
  sor.filter (*cloud);

  std::cout<<"Filtered Cloud Points "<< cloud->width * cloud->height<< std::endl;
  // END of Filtering
  



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

  pcl::SACSegmentation<PointT> seg;
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setDistanceThreshold(_max_distance);
  seg.setOptimizeCoefficients(true);
  seg.setMaxIterations(100);
  //seg.setEpsAngle(theta_eps);
  //seg.setAxis(plane_axis);

  // Declare extraction pointer
  pcl::ExtractIndices<PointT> extract; 

  //Declare vector of pointers containing all the segmented planes
  //std::vector <pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator <pcl::PointCloud<PointT>::Ptr>> v_pln_cloud;

  // Planar clustering loop
  printf ("Start planar clustering...\n");
  int nr_points = (int) cloud->size (); //compute size of initial point cloud to exit while()
  while (cloud->size() > _min_percentage * nr_points){

        // Fit a plane
        seg.setInputCloud(cloud);
        seg.segment(*inliers, *coefficients);

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

  // 2-EUCLIDIAN CLUSTER EXTRACTION
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (1); 
  ec.setMinClusterSize (20);
  ec.setMaxClusterSize (200);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  std::cout<< cluster_indices.size() <<" euclidian clusters found"<<std::endl;

  //Declare cloud vector, add each cluster to a new cloud vector slot
  //In visualization, generate a unique color for each cluster vector element 
  //and add to the visualizer

  // Extract points and copy them to clouds vector
  std::vector< pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr> > v_segment_clouds;
  pcl::PointCloud<PointT>::Ptr curr_segment_cloud;
  pcl::PointCloud<PointT>::Ptr res_cloud (new pcl::PointCloud<PointT>), res_cloud_temp (new pcl::PointCloud<PointT>);
  pcl::PointIndices::Ptr idx; //
 
  //1° try
 /*  for (int c =0; c<= cluster_indices.size(); c++)
  {
    std::cout<<"Saving cluster "<< c <<"...";
    curr_segment_cloud.reset (new pcl::PointCloud<PointT>);
    *idx = cluster_indices[c];
    
    //Extract segmented object
    extract.setInputCloud(cloud);
    extract.setIndices(idx);
    extract.setNegative(false);
    extract.filter(*curr_segment_cloud);

    // Push back point cloud into return vector
    v_segment_clouds.push_back(curr_segment_cloud);

    // extract rest of cloud (not segmented)
    //extract.setNegative(true);
    //extract.filter(*res_cloud_temp);
    //*res_cloud += *res_cloud_temp;
  
    std::cout<<"Cluster "<< c <<" saved."<<std::endl;
  }
  */
  /*
  // 3-CYLINDRICAL SEGMENTATION
  // Normal estimation (in OUR case, these will be handed out directly by the Zivid Camera)
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT> ());
  pcl::PointCloud<pcl::Normal>::Ptr obj_normals (new pcl::PointCloud<pcl::Normal>), obj_normals_temp (new pcl::PointCloud<pcl::Normal>);
  
  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud);
  ne.setKSearch (50);
  ne.compute (*obj_normals);
 

  // Create the segmentation object for cylinder segmentation and set all the parameters
  pcl::PointCloud<PointT>::Ptr cyl_cloud (new pcl::PointCloud<PointT>), res_cloud (new pcl::PointCloud<PointT>);
  pcl::PointIndices::Ptr cyl_inliers (new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr cyl_coefficients (new pcl::ModelCoefficients);
  
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg_n; //segmentation object from normals

  seg_n.setOptimizeCoefficients (true);
  seg_n.setModelType (pcl::SACMODEL_CYLINDER);
  seg_n.setMethodType (pcl::SAC_RANSAC);
  seg_n.setMaxIterations (10000);
  seg_n.setDistanceThreshold (0.2);
  seg_n.setRadiusLimits (0.1, 6);
  seg_n.setNormalDistanceWeight (0.1);
  seg_n.setInputCloud (cloud);
  seg_n.setInputNormals (obj_normals);

  // Obtain the cylinder inliers and coefficients
  seg_n.segment (*cyl_inliers, *cyl_coefficients);
  std::cerr << "Cylinder coefficients: " << *cyl_coefficients << std::endl;
  // Display info
  std::cerr << "Cylinder coefficients: "<< std::endl 
            << "Point on axis: "
            << "(" << cyl_coefficients->values[0]  
            << "," << cyl_coefficients->values[1] 
            << "," << cyl_coefficients->values[2] << ")" << std::endl 
            << "Axis direction: "
            << "(" << cyl_coefficients->values[3]  
            << "," << cyl_coefficients->values[4] 
            << "," << cyl_coefficients->values[5] << ")" << std::endl 
            << "Radius: " << cyl_coefficients->values[6]  << std::endl;

  // cylinder extraction
  extract.setInputCloud (cloud);
  extract.setIndices (cyl_inliers);
  extract.setNegative (false);
  extract.filter (*cyl_cloud);
  if (cyl_cloud->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else

  extract_normals.setNegative (true);
  extract_normals.setInputCloud (obj_normals);
  extract_normals.setIndices (cyl_inliers);
  extract_normals.filter (*obj_normals_temp);
  *obj_normals = *obj_normals_temp;

  extract.setNegative (true);
  extract.filter (*res_cloud);
  */

  //2° try
  int j = 0;
  for (const auto& cluster : cluster_indices)
  {
    pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    for (const auto& idx : cluster.indices) {
      cloud_cluster->push_back((*cloud)[idx]);
    } //*
    cloud_cluster->width = cloud_cluster->size ();
    cloud_cluster->height = 1;
    cloud_cluster->is_dense = true;

    std::cout<<"Saving cluster "<< j+1 <<" to point cloud cluster vector...";
    v_segment_clouds.push_back(cloud_cluster);

    std::cout<<"Cluster "<< j+1 <<" saved."<<std::endl;
    j++;
  }

  // END of SEGMENTATION


  // Visualization using PCLVisualizer
  pcl::visualization::PCLVisualizer viewer ("Cloud viewer");
  
  int v1(0);
  viewer.createViewPort (0.0, 0.0, 0.5, 1.0, v1);
  viewer.setBackgroundColor (0, 0, 0, v1);
  viewer.addText ("Source point cloud",10,10,"v1 text", v1);
  viewer.addPointCloud<PointT> (t_raw_cloud, "t_cloud", v1);

  int v2(0);
  viewer.createViewPort (0.5, 0.0, 1.0, 1.0, v2);
  viewer.setBackgroundColor (0.3, 0.3, 0.3, v2);
  viewer.addText ("Segmented point cloud", 10, 10, "v2 text", v2);
  //pcl::visualization::PointCloudColorHandlerCustom<PointT> obj_color (res_cloud, 255, 0, 0);
  pcl::visualization::PointCloudColorHandlerCustom<PointT> pln_color (pln_cloud, 0, 255, 0);
  
  //add objects point cloud (original color)
  //viewer.addPointCloud<PointT> (res_cloud, "obj_cloud", v2);
  
  //add extracted plane point cloud (shown in green)
  viewer.addPointCloud<PointT> (pln_cloud, pln_color, "pln_clouds", v2);

  //add point clouds from the segmented clouds vector (shown in random colors)
    std::stringstream cloud_name;
    int counter(0);
    pcl::RGB rgb;
    for (const auto &curr_cloud : v_segment_clouds) {
        ++counter;
        cloud_name.str("");
        cloud_name << "Segmentation " << counter;

        // Generate unique colour
        rgb = pcl::GlasbeyLUT::at(counter);

        // Create colour handle
        pcl::visualization::PointCloudColorHandlerCustom<PointT> colour_handle(curr_cloud, rgb.r, rgb.g, rgb.b);

        // Add points to viewer and set parameters
        viewer.addPointCloud<PointT> (curr_cloud, colour_handle, cloud_name.str());
        viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloud_name.str());
    }

  //add extracted cylinders (shown in red)
  // viewer.addCylinder(*cyl_coefficients, "cylinder", v2);
  // viewer.addPointCloud<PointT> (cyl_cloud, cyl_color, "cyl_clouds", v2);

  //add sphere in (0,0,0) with radius 500 for debugging filters
  //pcl::PointXYZ C(0,  0, 0);
  //viewer.addSphere (C, 500, 0, 0, 0.5, "sphere",v2);
  
  //visual utilities
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "t_cloud");
  //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "obj_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "pln_clouds");
  
  viewer.addCoordinateSystem (100);
  viewer.setBackgroundColor(255, 255, 255); // Setting background color

  while (!viewer.wasStopped ()) { // Display the visualiser 
  viewer.spinOnce ();
  }
  return (0);
}
