/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    
    const int renderCars          = 0;
    const int renderCloud         = 1;
    const int renderObst          = 2;
    const int renderPlane         = 3;
    const int renderObsAndPlane   = 4;
    const int renderCluster       = 5;
    const int render_Box          = 6;
    const int renderClusterAndBox = 7;
    const int renderClusterAndBoxAndPlane = 8;

    int renderOption = renderClusterAndBoxAndPlane;
    
    switch(renderOption)
    {
        case renderCars:
            renderScene = true;
            break;
        case renderCloud:
        case renderObst:
        case renderPlane:
        case renderObsAndPlane:
        case renderCluster:
        case render_Box:
        case renderClusterAndBox:
        case renderClusterAndBoxAndPlane:
            renderScene = false;
            break;
        default:
            renderScene = true;
            renderOption = renderCars;
            break;
    }

    std::vector<Car> cars = initHighway(renderScene, viewer);
    Color color = Color(1,1,1);

    // TODO:: Create lidar sensor 
    Lidar* lidar = new Lidar(cars, 0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud = lidar->scan();

    // TODO:: Create point processo
    // point cloud segmentation
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segmentCloud = pointProcessor.SegmentPlane(inputCloud, 100, 0.2);

    //obstacles clustering
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 1.0, 3, 30);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    switch(renderOption)
    {
        case renderCars:
            renderRays(viewer, lidar->position, inputCloud);
            break;
        case renderCloud:
            renderPointCloud(viewer, inputCloud, "inputCloud");
            break;
        case renderObst:
            renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
            break;
        case renderPlane:
            renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));
            break;
        case renderObsAndPlane:
            renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
            renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));
            break;
        case renderCluster:
            for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
            {
                std::cout << "cluster size "; // this line is just an output string in the terminal
                pointProcessor.numPoints(cluster);
                renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId%colors.size()]);
                ++clusterId;
            }
            break;
        case render_Box:
            for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
            {
                Box box = pointProcessor.BoundingBox(cluster);
                renderBox(viewer,box,clusterId);

                ++clusterId;
            }
            break;
        case renderClusterAndBox:
            for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
            {
                std::cout << "cluster size "; // this line is just an output string in the terminal
                pointProcessor.numPoints(cluster);
                renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId%colors.size()]);
                
                Box box = pointProcessor.BoundingBox(cluster);
                renderBox(viewer,box,clusterId);
                
                ++clusterId;
            }
            break;
        case renderClusterAndBoxAndPlane:
            for(pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
            {
                std::cout << "cluster size "; // this line is just an output string in the terminal
                pointProcessor.numPoints(cluster);
                renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId%colors.size()]);
                
                Box box = pointProcessor.BoundingBox(cluster);
                renderBox(viewer,box,clusterId);
                
                ++clusterId;
            }

            renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));
            break;
        default:
            break;
    }
}

void cityBlockStatic(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // Open 3D viewer and display City Block
    
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");

    //FIlter point cloud
    pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud = pointProcessorI->FilterCloud(inputCloud, 0.1 , Eigen::Vector4f (-15.0, -6.0, -2.5, 1), Eigen::Vector4f(30.0, 6.5, 2.5, 1));
    
    // point cloud segmentation
    ProcessPointClouds<pcl::PointXYZI> pointProcessor;
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessor.SegmentPlane(filterCloud, 100, 0.2);
    
    //obstacles clustering
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessor.Clustering(segmentCloud.first, 0.5, 10, 800);
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    //Render full point cloud without filter
    //renderPointCloud(viewer, inputCloud, "inputCloud");

    //Render point cloid filtered
    //renderPointCloud(viewer, filterCloud, "filterCloud");

    //renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0));
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));
    
    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size "; // this line is just an output string in the terminal
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obstCloud"+std::to_string(clusterId), colors[clusterId%colors.size()]);
        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer,box,clusterId);

        ++clusterId;
    }
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer, ProcessPointClouds<pcl::PointXYZI> pointProcessorI, pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud)
{
    //FIlter point cloud
    filterCloud = pointProcessorI.FilterCloud(filterCloud, 0.1 , Eigen::Vector4f (-15.0, -6.0, -2.5, 1), Eigen::Vector4f(30.0, 6.5, 2.5, 1));
    
    // point cloud segmentation
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentCloud = pointProcessorI.RansacSegmentation(filterCloud, 50, 0.2);

    //obstacles clustering
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters = pointProcessorI.Clustering(segmentCloud.first, 0.5, 10, 800);
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(1,0,0), Color(1,1,0), Color(0,0,1)};

    for(pcl::PointCloud<pcl::PointXYZI>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size "; // this line is just an output string in the terminal
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
    int streamSelector = 0;
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    //cityBlock(viewer);

    ProcessPointClouds<pcl::PointXYZI> pointProcessorI;// = new ProcessPointClouds<pcl::PointXYZI>();
	  
    std::vector<boost::filesystem::path> stream = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_1");
    std::vector<boost::filesystem::path> stream2 = pointProcessorI.streamPcd("../src/sensors/data/pcd/data_2");
  	auto streamIterator = stream.begin();
  	auto streamIterator2 = stream2.begin();
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped ())
    {
        // Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();
        if ( streamSelector == 0 )
        {
            // Load pcd and run obstacle detection process
            inputCloudI = pointProcessorI.loadPcd((*streamIterator).string());
            cityBlock(viewer, pointProcessorI, inputCloudI);

            streamIterator++;

            if (streamIterator == stream.end())
            {
                streamIterator = stream.begin();
                streamSelector = 1;
            }
        }
        else if ( streamSelector == 1 )
        {
            // Load pcd and run obstacle detection process
            inputCloudI = pointProcessorI.loadPcd((*streamIterator2).string());
            cityBlock(viewer, pointProcessorI, inputCloudI);

            streamIterator2++;

            if (streamIterator2 == stream2.end())
            {
                streamIterator2 = stream2.begin();
                streamSelector = 0;
            }
        }
        else
        {
            streamIterator = stream.begin();
            streamIterator2 = stream2.begin();
            streamSelector = 0;
            
        }
        
        

        viewer->spinOnce ();
    } 
}