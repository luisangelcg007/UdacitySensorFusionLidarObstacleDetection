/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "processPointClouds.h"
#include "sensors/lidar.h"
#include "render/render.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud)
{
    /*Segmentation parameters*/
    int maxIterationsToSegmentation = 50; 
    float distanceToleranceToSegmentation = 0.2;

    /*Clustering parameters*/
    float distanceClusterTolerance = 0.3;
    int minSizeNumberOfPoint = 40;
    int maxSizeNumberOfPoints = 2000;

    /*FIlter point cloud*/
    filterCloud = pointProcessorI.FilterCloud(filterCloud, 0.1 , Eigen::Vector4f (-15.0, -6.0, -2.5, 1), Eigen::Vector4f(30.0, 6.5, 2.5, 1));
    
    /*Point cloud segmentation*/
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = 
        pointProcessorI.RansacSegmentation(
            filterCloud, 
            maxIterationsToSegmentation, 
            distanceToleranceToSegmentation);

    //obstacles clustering
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = 
        pointProcessorI.kdTreeClustering(
            segmentCloud.first, 
            distanceClusterTolerance, 
            minSizeNumberOfPoint, 
            maxSizeNumberOfPoints);
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        pointProcessorI.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId%colors.size()]);
        Box box = pointProcessorI.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);

        ++clusterId;
    }
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

int main (int argc, char** argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;
	  
    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
  	auto streamIterator = stream.begin();
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        
        /* Load pcd and run obstacle detection process */
        inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());
        cityBlock(viewer, pointProcessorI, inputCloudI);

        streamIterator++;

        if (streamIterator == stream.end())
        {
            streamIterator = stream.begin();
        }
        viewer->spinOnce ();
    } 
}