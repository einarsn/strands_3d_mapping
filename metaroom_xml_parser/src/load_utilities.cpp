#include "load_utilities.h"
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
using namespace cv;

typedef pcl::PointXYZRGB PointType;

boost::shared_ptr<pcl::visualization::PCLVisualizer> rgbVis (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);

int main(int argc, char** argv)
{

    std::string pathToSweepXml = "/home/einar/catkin_ws/data/KTH_longterm_dataset/20140820/patrol_run_2/room_0/room.xml";

    IntermediateCloudCompleteData<PointType> test = loadIntermediateCloudsCompleteDataFromSingleSweep<PointType>(pathToSweepXml);

    boost::shared_ptr<pcl::PointCloud<PointType>> mergedCloudPtr = loadMergedCloudFromSingleSweep<PointType>(pathToSweepXml);
    boost::shared_ptr<pcl::PointCloud<PointType>> slicedCloudPtr (new pcl::PointCloud<PointType>);
;
    /*
    ----------- EXTRACT A SLICE OUT OF THE MERGED CLOUD -----------
    */
    pcl::PassThrough<PointType> pass;
    pass.setInputCloud (mergedCloudPtr);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.2, 0.3);
    pass.filter (*slicedCloudPtr);

    int size = test.vIntermediateRGBImages.size();
/*    namedWindow( "RGB", WINDOW_AUTOSIZE );// Create a window for display.*/
    //namedWindow( "DEPTH", WINDOW_AUTOSIZE );// Create a window for display.
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer;
    viewer = rgbVis(slicedCloudPtr);

/*    for(int i = 0; i < size; i++)
    {
        imshow( "RGB", test.vIntermediateRGBImages[i] );                   // Show our image inside it.
        //imshow( "DEPTH", test.vIntermediateDepthImages[i] );                   // Show our image inside it.
        //viewer = rgbVis(test.vIntermediateRoomClouds[i]);
        waitKey(0);
    }*/
    while (!viewer->wasStopped () )
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
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
