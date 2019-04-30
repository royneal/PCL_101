#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/mls.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>
ros::Publisher pub;

void cloud_cb (const sensor_msgs::PointCloud2ConstPtr& input)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;

  // Do data processing here...
  output = *input;

  // Publish the data.
  pub.publish (output);
}

int main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;


  // Load file | Works with PCD and PLY files
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud (new pcl::PointCloud<pcl::PointXYZRGB> ());

// read point cloud
    pcl::io::loadPLYFile ("/home/royneal/pcl_ws/src/box.ply", *source_cloud);



    // Load bun0.pcd -- should be available with the PCL archive in test


    // Create a KD-Tree
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGB>);

    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::PointCloud<pcl::PointNormal> mls_points;
    pcl::PointCloud<pcl::PointNormal>::Ptr Normal_cloud (new pcl::PointCloud<pcl::PointNormal> ());
    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;

    mls.setComputeNormals (true);

    // Set parameters
    mls.setInputCloud (source_cloud);
    mls.setPolynomialOrder (2);
    mls.setSearchMethod (tree);
    mls.setSearchRadius (0.03);

    // Reconstruct
    mls.process (mls_points);
    *Normal_cloud = mls_points;

    ROS_INFO( "Originl cloud: %d Normals cloud: %d", source_cloud->points.size(), mls_points.points.size());
    pcl::visualization::PCLVisualizer viewer ("box");
     // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> source_cloud_color_handler (source_cloud, 255, 255, 255);
     // We add the point cloud to the viewer and pass the color handler
     //viewer.addPointCloud (source_cloud, source_cloud_color_handler, "original_cloud");
  //    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(source_cloud);
  // viewer.addPointCloud<pcl::PointXYZRGB> (source_cloud, rgb, "sample cloud");

      pcl::visualization::PointCloudColorHandlerCustom<pcl::PointNormal> transformed_cloud_color_handler (Normal_cloud, 230, 20, 20); // Red
       viewer.addPointCloud (Normal_cloud, transformed_cloud_color_handler, "transformed_cloud");
        // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
       // viewer.addPointCloudNormals<pcl::PointXYZRGB, pcl::PointNormal> (source_cloud, Normal_cloud, 10, 0.05, "normals");

       // viewer.addPointCloudNormals <pcl::PointXYZRGB, pcl::PointNormal> (source_cloud, Normal_cloud, 30, 0.1, "normals", 0);
       // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 4, "normals");
       // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, 1.0, 0.0, 0.0, "normals");

     viewer.addCoordinateSystem (1.0, "sample cloud", 0);
     viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
    // viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "original_cloud");

      //viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2, "normals");
      //viewer.setPosition(800, 400); // Setting visualiser window position

      // pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
      // viewer->setBackgroundColor (0, 0, 0);
      // pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
      // viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
      // viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
      // viewer->addPointCloudNormals<pcl::PointXYZRGB, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
      viewer.addCoordinateSystem (1.0);
      viewer.initCameraParameters ();






      while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
        viewer.spinOnce ();
      }
  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("input", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("output", 1);

  // Spin
  ros::spin ();
}
