//PCD CLUSTERING TESTING

//utility libraries
#include <iostream>
#include <Eigen/Geometry>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/distances.h>
#include <pcl/common/time.h>
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
#include <pcl/filters/project_inliers.h>

typedef pcl::PointXYZRGBA PointT;

int main()
{
  pcl::StopWatch watch;
  watch.reset();

  // Load PCD file
  pcl::PointCloud<PointT>::Ptr raw_cloud (new pcl::PointCloud<PointT>);

  std::string path = "/Users/marcobogataj/Documents/UNI/magistrale/BG/THESIS/Tesi ABB/zivid_captures/";
  if (pcl::io::loadPLYFile<PointT> (path+std::string("zivid_manual_holefilling.ply"), *raw_cloud) == -1) //* load the file
  {
      PCL_ERROR ("Couldn't read the .PCD file\n");
      return (-1);
  }

  std::cout<<"Source Cloud Points "<< raw_cloud->width * raw_cloud->height<< std::endl;

  std::cout<<std::endl<<"Loading time "<< watch.getTimeSeconds() << "seconds" <<std::endl;

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

  std::cout<<std::endl<<"Transform time "<< watch.getTimeSeconds() << "seconds" <<std::endl<<std::endl;


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
  
  std::cout<<std::endl<<"Filtering time "<< watch.getTimeSeconds() << "seconds" <<std::endl<<std::endl;


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

  // Planar clustering loop
  //POSSIBLE UPGRADES:
  //Terminate not by checking percentage clustered but by checking size of last plane clustered.
  //idea: Exit if size of last plane in terms of points is significantly smaller than a threshold 
  //(either relative (to previous planes), or absolute (to planes that I usually find because of tables, containers, boxes))

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
  printf ("Planar clustering iteration finished.\n");
  std::cout<< "Number of planes segmented: " << n_planes <<std::endl<<std::endl;

  std::cout<<std::endl<<"Planar segmentation time "<< watch.getTimeSeconds() << "seconds" <<std::endl<<std::endl;
  
  // 2-EUCLIDIAN CLUSTER EXTRACTION
  // Creating the KdTree object for the search method of the extraction
  pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
  tree->setInputCloud (cloud);

  std::vector<pcl::PointIndices> cluster_indices;
  pcl::EuclideanClusterExtraction<PointT> ec;
  ec.setClusterTolerance (0.6); 
  ec.setMinClusterSize (2000);
  ec.setMaxClusterSize (28000);
  ec.setSearchMethod (tree);
  ec.setInputCloud (cloud);
  ec.extract (cluster_indices);

  std::cout<< cluster_indices.size() <<" euclidian clusters found"<<std::endl;

  //Declare cloud vector, add each cluster to a new cloud vector slot
  //In visualization, generate a unique color for each cluster vector element 
  //and add to the visualizer

  // Extract points and copy them to clouds vector -> v_segment_clouds
  std::vector< pcl::PointCloud<PointT>::Ptr, Eigen::aligned_allocator<pcl::PointCloud<PointT>::Ptr> > v_segment_clouds;
  pcl::PointCloud<PointT>::Ptr curr_segment_cloud;
  pcl::PointCloud<PointT>::Ptr res_cloud (new pcl::PointCloud<PointT>), res_cloud_temp (new pcl::PointCloud<PointT>);
  pcl::PointIndices::Ptr idx; 

  //define objects for centroid extraction
  std::vector<pcl::PointXYZ, Eigen::aligned_allocator <pcl::PointXYZ> > v_centroids;
  Eigen::Vector4f centroid;
  pcl::PointXYZ centroid_point;
 
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

    //std::cout<<"Saving cluster "<< j+1 <<" to point cloud cluster vector..."<<std::endl;
    v_segment_clouds.push_back(cloud_cluster);

    //std::cout<<"Compute and save cluster centroid..."<<std::endl;
    pcl::compute3DCentroid(*cloud_cluster, centroid); 

    //First three elements of centroid are x,y,x. Last is 1 to allow 4x4 matrix transformations
    //Save centroid as PointXYZ
    centroid_point.x = centroid[0];
    centroid_point.y = centroid[1];
    centroid_point.z = centroid[2];

    v_centroids.push_back(centroid_point); //add to vector of centroids

    std::cout<<"Cluster "<< j+1 <<" size: "<< cloud_cluster->size() <<std::endl;
    j++;
  }

  std::cout<<std::endl<<"Eucledian clusterization time "<< watch.getTimeSeconds() << "seconds" <<std::endl<<std::endl;

  // 3-CONDITIONAL EUCLIDIAN CLUSTER EXTRACTION
  //{TO DO!!!}


  // END of SEGMENTATION

  // FIT PRIMITIVE MODELS on clusters
  // Fit cylinder

  // Idea: 
  // use pcl::SACSegmentationFromNormals with setModelType (pcl::SACMODEL_CYLINDER) to estimate
  // cylinder coefficients point_on_axis (𝑐), axis_direction (𝑣), radius (R).
  pcl::SACSegmentationFromNormals<PointT, pcl::Normal> seg2;
  pcl::PointIndices::Ptr inliers_cylinder (new pcl::PointIndices);
  std::vector<pcl::ModelCoefficients> v_coefficients_cylinder;
  pcl::ModelCoefficients::Ptr coefficients_cyl_temp (new pcl::ModelCoefficients);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::NormalEstimation<PointT, pcl::Normal> ne;
  pcl::search::KdTree<PointT>::Ptr tree2 (new pcl::search::KdTree<PointT> ());
  std::vector<float> h; //cylinder heigh estimation
  std::vector<int> cyl_found; //1-> cyl found, 0->cyl not found
  //std::vector<PointT> p_min, p_max;
  pcl::PointCloud<PointT>::Ptr line_proj (new pcl::PointCloud<PointT>);


  //ITERATE CLUSTERS
  std::cout <<std::endl << "Start cylindrical fitting"<< "..." << std::endl<<endl;;

  int i=0;
  for(auto& segment : v_segment_clouds)
  {
    // Estimate point normals
    ne.setSearchMethod (tree2);
    ne.setInputCloud (segment);
    ne.setKSearch (50);
    ne.compute (*cloud_normals);

    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg2.setOptimizeCoefficients (true);
    seg2.setModelType (pcl::SACMODEL_CYLINDER);
    seg2.setMethodType (pcl::SAC_RANSAC);
    seg2.setNormalDistanceWeight (0.1);
    seg2.setMaxIterations (5000);
    seg2.setDistanceThreshold (2);
    seg2.setRadiusLimits (1,10);
    seg2.setInputCloud (segment);
    seg2.setInputNormals (cloud_normals);

    // Obtain the cylinder inliers and coefficients
    seg2.segment (*inliers_cylinder, *coefficients_cyl_temp);
    std::cerr << "Compute cylinder coefficients for cluster "<<i+1<< "..." << std::endl;

    extract.setInputCloud (segment);
    extract.setIndices (inliers_cylinder);
    extract.setNegative (false);
    pcl::PointCloud<PointT>::Ptr cloud_cylinder (new pcl::PointCloud<PointT> ());
    extract.filter (*cloud_cylinder);

    int size_cyl = cloud_cylinder->width*cloud_cylinder->height;

    if ((cloud_cylinder->points.empty ()) || (cloud_cylinder->size() < segment->size()*0.70)) //0.7 parameter to be tuned!
    {
      cyl_found.push_back(0); //not found 

      std::cout << "Can't find the cylindrical component." << std::endl<<std::endl;
    }
    else
    { 
      cyl_found.push_back(1); //found

      // Display cylinder info
      std::cout << "Point on axis  ->(" << coefficients_cyl_temp->values[0] << ", " 
                                        << coefficients_cyl_temp->values[1] << ", "
                                        << coefficients_cyl_temp->values[2] << ") "<<std::endl<< 
                  "Axis direction ->("  << coefficients_cyl_temp->values[3] << ", "
                                        << coefficients_cyl_temp->values[4] << ", "
                                        << coefficients_cyl_temp->values[5] << ") "<<std::endl<< 
                  "Radius = "           << coefficients_cyl_temp->values[6] <<std::endl;

      std::cout<< "Cylinder inlier points:" << cloud_cylinder->size() <<std::endl<<std::endl;

      // To estimate cylinder height: https://math.stackexchange.com/questions/3324579/sorting-collinear-points-on-a-3d-line
    // 1-> Project cylinder inliers onto the cylinder axis 𝑣. (https://pcl.readthedocs.io/projects/tutorials/en/latest/project_inliers.html)
    // 2-> Select first and last points in the cylinder point cloud (ONLY with ordered point clouds file types)

      pcl::ModelCoefficients::Ptr coefficients_line (new pcl::ModelCoefficients ());
      coefficients_line->values.resize (6);
      
      for(int k=0; k<=5; k++) coefficients_line->values[k] = coefficients_cyl_temp->values[k];

      std::cout << "Point on axis ->("  << coefficients_line->values[0] << ", " 
                                        << coefficients_line->values[1] << ", "
                                        << coefficients_line->values[2] << ") "<<std::endl<< 
                  "Axis direction ->("  << coefficients_line->values[3] << ", "
                                        << coefficients_line->values[4] << ", "
                                        << coefficients_line->values[5] << ") "<<std::endl;

      pcl::ProjectInliers<PointT> proj;
      proj.setModelType (pcl::SACMODEL_LINE);
      proj.setInputCloud (cloud_cylinder);
      proj.setModelCoefficients (coefficients_line);
      proj.filter (*line_proj);

      std::cout<< "Cylinder inlier points:" << line_proj->size() <<std::endl;

      //compute height by computing the segment connecting first and last points of the cylinder inliers
      //since the point cloud is organized and saved in a ordered fashion, these results in the two extremes on the cyl axis.
      std::cout<<"Pmin="<<line_proj->points[0]<<std::endl;
      std::cout<<"Pmax="<<line_proj->points[line_proj->size()-1]<<std::endl<<std::endl;

      h.push_back(pcl::euclideanDistance(line_proj->points[0],line_proj->points[line_proj->size()-1])); //cylinder height estimation

      coefficients_cyl_temp->values[0]=line_proj->points[0].x;
      coefficients_cyl_temp->values[1]=line_proj->points[0].y;
      coefficients_cyl_temp->values[2]=line_proj->points[0].z;
      coefficients_cyl_temp->values[3]=line_proj->points[line_proj->size()-1].x - line_proj->points[0].x;
      coefficients_cyl_temp->values[4]=line_proj->points[line_proj->size()-1].y - line_proj->points[0].y;
      coefficients_cyl_temp->values[5]=line_proj->points[line_proj->size()-1].z - line_proj->points[0].z;

      v_coefficients_cylinder.push_back(*coefficients_cyl_temp);
    }
    i++;
  }
  std::cout<<std::endl<<"Cylindrical fitting time: "<< watch.getTimeSeconds() << "seconds" <<std::endl<<std::endl;

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

  pcl::visualization::PointCloudColorHandlerCustom<PointT> pln_color (pln_cloud, 0, 255, 0);
  
  //add extracted plane point cloud (shown in green)
  viewer.addPointCloud<PointT> (pln_cloud, pln_color, "pln_clouds", v2);

  //add point clouds from the segmented clouds vector (shown in random colors)
  std::stringstream cloud_name, centroid_text, centroid_sphere, cylinder_name, cyl_axis;
  int counter(0);
  int cyl_counter(0);
  pcl::RGB rgb;
  for (const auto &curr_cloud : v_segment_clouds) {
      
      cloud_name.str("");
      centroid_text.str("");
      centroid_sphere.str("");
      cylinder_name.str("");
      cyl_axis.str("");
    

      cloud_name << "Segmentation " << counter;
      centroid_text <<"Cluster " <<counter;
      centroid_sphere <<"C" <<counter;
      cylinder_name <<"Cylinder" <<cyl_counter;
      cyl_axis <<"cyl_axis "<<cyl_counter;
  

      // Generate unique colour
      rgb = pcl::GlasbeyLUT::at(counter);

      // Create colour handle
      pcl::visualization::PointCloudColorHandlerCustom<PointT> colour_handle(curr_cloud, rgb.r, rgb.g, rgb.b);

      // Add points to viewer and set parameters

      if (cyl_found[counter] == 0) //add as cluster
      {
        viewer.addText3D(centroid_text.str(),v_centroids[counter], 2.0, 0.0, 0.0, 0.0, centroid_text.str(), v2);
        viewer.addSphere (v_centroids[counter], 2, 0, 0, 0,centroid_sphere.str(),v2);
      }
      else //add as cylinder
      {
        viewer.addText3D(cylinder_name.str(),v_centroids[counter], 2.0, 0.0, 0.0, 0.0, cylinder_name.str(), v2);
        viewer.addCylinder(v_coefficients_cylinder[cyl_counter], cylinder_name.str(), v2);
        std::cout<<v_coefficients_cylinder[cyl_counter]<<std::endl;
        cyl_counter++;
      }

      viewer.addPointCloud<PointT> (curr_cloud, colour_handle, cloud_name.str(),v2);
      viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, cloud_name.str());
      counter++;
  }  

  //add sphere in (0,0,0) with radius 500 for debugging filters
  //pcl::PointXYZ C(0,0,0);
  //viewer.addSphere (C, 500, 0, 0, 0.5, "sphere",v2);
  
  //visual utilities
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "t_cloud");
  viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "pln_clouds");
  
  viewer.addCoordinateSystem (10);
  viewer.setBackgroundColor(255, 255, 255); // Setting background color

  std::cout<<std::endl<<"Setting up visualization time: "<< watch.getTimeSeconds() << "seconds" <<std::endl<<std::endl;

  while (!viewer.wasStopped ()) { // Display the visualiser 
  viewer.spinOnce ();
  }
  return (0);
}
