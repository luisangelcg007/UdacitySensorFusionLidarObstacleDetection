// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"


//constructor:
template<typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}


//de-constructor:
template<typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}


template<typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::VoxelGrid<PointT> vg;
  	typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
  
  	vg.setInputCloud(cloud);
  	vg.setLeafSize(filterRes, filterRes, filterRes);
  	vg.filter(*cloudFiltered);
  
  	typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
  
  	pcl::CropBox<PointT> region(true);
  	region.setMin(minPoint);
  	region.setMax(maxPoint);
  	region.setInputCloud(cloudFiltered);
  	region.filter(*cloudRegion);
  
  	std::vector<int> indices;

  	pcl::CropBox<PointT> roof(true);
  	roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
  	roof.setMax(Eigen::Vector4f(2.6, 1.7, -0.3, 1));
  	roof.setInputCloud(cloudRegion);
  	roof.filter(indices);
  
  	pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
  	for (int point : indices)
    	inliers->indices.push_back(point);
  
  	pcl::ExtractIndices<PointT> extract;
  	extract.setInputCloud(cloudRegion);
  	extract.setIndices(inliers);
  	extract.setNegative(true);
  	extract.filter(*cloudRegion);
  
    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    
    return cloudRegion;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstCloud (new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr planeCloud (new pcl::PointCloud<PointT>());

    for(int index : inliers->indices)
    {
        planeCloud->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloud);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*obstCloud);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstCloud, planeCloud);
    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloud, cloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	//pcl::PointIndices::Ptr inliers;
    // TODO:: Fill in this function to find inliers for the cloud.
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    //Segment the largest planar component from the input cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);
    if(inliers->indices.size() == 0)
    {
        std::cout << "Could not estimatea planar model for the given dataset." << std::endl;
    }

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    return segResult;
}


template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    tree->setInputCloud (cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // 2cm
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud);
    ec.extract (clusterIndices);

    for (pcl::PointIndices getIndices: clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        
        for (int index : getIndices.indices)
        {
            cloudCluster->points.push_back (cloud->points[index]); //*
        }
        cloudCluster->width = cloudCluster->points.size ();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;

    return clusters;
}

template<typename PointT>
Box ProcessPointClouds<PointT>::BoundingBox(typename pcl::PointCloud<PointT>::Ptr cluster)
{

    // Find bounding box for one of the clusters
    PointT minPoint, maxPoint;
    pcl::getMinMax3D(*cluster, minPoint, maxPoint);

    Box box;
    box.x_min = minPoint.x;
    box.y_min = minPoint.y;
    box.z_min = minPoint.z;
    box.x_max = maxPoint.x;
    box.y_max = maxPoint.y;
    box.z_max = maxPoint.z;

    return box;
}

template<typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII (file, *cloud);
    std::cerr << "Saved " << cloud->points.size () << " data points to "+file << std::endl;
}


template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT> (file, *cloud) == -1) //* load the file
    {
        PCL_ERROR ("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+file << std::endl;

    return cloud;
}

template<typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{

    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});

    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;

}

template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::RansacSegmentation(
    typename pcl::PointCloud<PointT>::Ptr cloud, 
    int maxIterations, 
    float distanceTolerance)
{
    std::unordered_set<int> inliersResultSegmentation;
	typename pcl::PointCloud<PointT>::Ptr roadCloud(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr obstaclesCloud(new pcl::PointCloud<PointT>());

    auto startTime = std::chrono::steady_clock::now();
    srand(time(NULL));
	
	for(int i = 0; i < maxIterations; i++) 
    {		
		std::unordered_set<int> inliers;
		
		while (inliers.size() < 3) 
        {
			int random_point = (static_cast<double>(std::rand()) / RAND_MAX) * cloud->points.size();
			inliers.insert(random_point);
		}

		auto iterator = inliers.begin();
		
		pcl::PointXYZI p1 = cloud->points[*iterator];
		float x1 = p1.x;
		float y1 = p1.y;
		float z1 = p1.z;
		++iterator;

		pcl::PointXYZI p2 = cloud->points[*iterator];
		float x2 = p2.x;
		float y2 = p2.y;
		float z2 = p2.z;
		++iterator;
		
		pcl::PointXYZI p3 = cloud->points[*iterator];
		float x3 = p3.x;
		float y3 = p3.y;
		float z3 = p3.z;

		std::tuple<float, float, float> normal
        (
            (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1),
			(z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1),
			(x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1)
        );

		float A = std::get<0>(normal);
		float B = std::get<1>(normal);
		float C = std::get<2>(normal);
		float D = -(A * x1 + B * y1 + C * z1);
		float E = (A * A) + (B * B) + (C * C);
		
		for(int i = 0; i < cloud->points.size(); i++)
        {
			pcl::PointXYZI p0 = cloud->points[i];

			if (inliers.count(i) > 0)
            {
				continue;
            }

			float shortestDistance = std::fabs(A * p0.x + B * p0.y + C * p0.z + D) / std::sqrt(E);
			
			if (shortestDistance <= distanceTolerance) 
            {
				inliers.insert(i);
			}
		}
		if (inliers.size() > inliersResultSegmentation.size()) 
        {
			inliersResultSegmentation = inliers;
		}
	}

	for(int index = 0; index < cloud->points.size(); index++)
	{
		pcl::PointXYZI point = cloud->points[index];
		if(inliersResultSegmentation.count(index))
        {
			roadCloud->points.push_back(point);
        }
		else
        {
			obstaclesCloud->points.push_back(point);
        }
	}

    auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    return std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr>(obstaclesCloud, roadCloud);
}

static void euclideanClusterHelper(
    int indice, 
    const std::vector<std::vector<float>>& points, 
    std::vector<int>& cluster, 
    std::vector<bool>& processed, 
    KdTree* tree, float distanceTol)
{
	processed[indice] = true;

	cluster.push_back(indice);

	std::vector<int> nearest = tree->search(points[indice], distanceTol);

	for (int id : nearest)
	{
		if (!processed[id])
			euclideanClusterHelper(id, points, cluster, processed, tree, distanceTol);
	}
}

static std::vector<std::vector<int>> euclideanCluster(
    const std::vector<std::vector<float>>& points, 
    KdTree* tree, 
    float distanceTol)
{
	std::vector<std::vector<int>> clusters;
 
	std::vector<bool> processed(points.size(), false);

	int i = 0;
	while (i < points.size())
	{
		if (processed[i])
		{
			i++;
			continue;
		}

		std::vector<int> cluster;
		euclideanClusterHelper(i, points, cluster, processed, tree, distanceTol);
		clusters.push_back(cluster);
		i++;
	}

	return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::kdTreeClustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, 
    float clusterTolerance, 
    int minSize, 
    int maxSize)
{
    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    KdTree* tree = new KdTree;
    std::vector<std::vector<float>> points;
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
 
    for (int i=0; i<cloud->points.size(); i++)
    {
        pcl::PointXYZI pointCloud = cloud->points[i];

        const std::vector<float> point{ pointCloud.x, pointCloud.y, pointCloud.z };
    	tree->insert(point,i);
        points.push_back(point);
    }
  
    std::vector<std::vector<int>> clusterIndices = euclideanCluster(points, tree, clusterTolerance);

    for (auto indices : clusterIndices)
    {
        if (indices.size() < minSize || indices.size() > maxSize) 
        { 
            continue;
        }

        typename pcl::PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);

        for (int index : indices)
        {
            cloudCluster->points.push_back (cloud->points[index]);
        }
        cloudCluster->width = cloudCluster->points.size ();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);

    return clusters;
}