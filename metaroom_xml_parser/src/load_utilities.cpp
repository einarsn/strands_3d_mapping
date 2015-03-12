#include "load_utilities.h"
#include <math.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/parse.h>
#include <pcl/filters/passthrough.h>

using namespace semantic_map_load_utilties;

typedef pcl::PointXYZRGB PointType;
typedef std::vector<boost::shared_ptr<pcl::PointCloud<PointType>>> dCLvector;

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

int main(int argc, char** argv)
{
    pcl::PointCloud<PointType>::Ptr cloud (new pcl::PointCloud<PointType>);
    pcl::PointCloud<PointType>::Ptr cloud_filtered (new pcl::PointCloud<PointType>);


    if (pcl::io::loadPCDFile<PointType> ("/home/einar/catkin_ws/data/KTH_longterm_dataset_processed/20140820/patrol_run_2/room_1/dynamic_clusters.pcd", *cloud) == -1) //* load the file
    {
      PCL_ERROR ("Couldn't read file test_pcd.pcd \n");
      return (-1);
    }

    pcl::PassThrough<PointType> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.35, 0.45);
    pass.filter (*cloud_filtered);

    float x_max=cloud->points[0].x;
    float y_max=cloud->points[0].y;
    float x_min=cloud->points[0].x;
    float y_min=cloud->points[0].y;
    for(int i = 1; i < cloud->points.size(); i++)
    {
        if(x_max < cloud->points[i].x)
            x_max = cloud->points[i].x;
        if(x_min > cloud->points[i].x)
            x_min = cloud->points[i].x;
        if(y_max < cloud->points[i].y)
            y_max = cloud->points[i].y;
        if(y_min > cloud->points[i].y)
            y_min = cloud->points[i].y;
    }

    for (size_t i = 0; i < cloud_filtered->points.size (); ++i)
      std::cout << "    " << cloud_filtered->points[i].x
                << " "    << cloud_filtered->points[i].y
                << " "    << cloud_filtered->points[i].z <<  "         " << i << std::endl;

    cout << "x_max: " << x_max << " x_min: " << x_min << endl;
    cout << "y_max: " << y_max << " y_min: " << y_min << endl;

    int x_translation, y_translation;
    if(x_min < 0)
        x_translation = fabs(floor(x_min));
    if(y_min < 0)
        y_translation = fabs(floor(y_min));

    std::vector<std::pair<float,float>> dynamic_points;

    for(int i = 0; i < cloud->points.size(); i++)
    {
        float x,y;
        x = cloud->points[i].x + x_translation;
        y = cloud->points[i].y + y_translation;
        dynamic_points.push_back(std::make_pair(x,y));
    }





    return 0;
}
boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  // --------------------------------------------
  // -----Open 3D viewer and add point cloud-----
  // --------------------------------------------
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(cloud);
  viewer->addPointCloud<pcl::PointXYZRGB> (cloud, rgb, "sample cloud");
  viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
  viewer->addCoordinateSystem (1.0);
  viewer->initCameraParameters ();
  return (viewer);
}
