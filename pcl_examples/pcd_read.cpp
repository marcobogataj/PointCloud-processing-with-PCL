#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

int main()
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);

    std::string path2cloud = "/Users/marcobogataj/Documents/UNI/magistrale/BG/THESIS/Tesi ABB/zivid_captures/zivid_manual_holefilling.pcd";
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB> (path2cloud, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
        return (-1);
    }
    // Visualization
    pcl::visualization::PCLVisualizer viewer ("Cloud viewer");

    // We add the point cloud to the viewer and pass the color handler
    viewer.addPointCloud (cloud, "original_cloud");
    viewer.addCoordinateSystem (1.0, "cloud", 0);
    viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey

    while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
    return 0;
    }
}

