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

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr cloudfiltered (new pcl::PointCloud<PointT>);
    pcl::VoxelGrid<PointT> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (filterRes,filterRes,filterRes);
    vg.filter (*cloudfiltered);

    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> cb (true);
    cb.setMax (maxPoint);
    cb.setMin (minPoint);
    cb.setInputCloud (cloudfiltered);
    cb.filter (*cloudRegion);

    std::vector<int> indices;
    pcl::CropBox<PointT> roof (true);
    roof.setMax (Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMin (Eigen::Vector4f (2.6, 1.7, -0.4, 1));
    roof.setInputCloud (cloudRegion);
    roof.filter (indices);

    pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    for (auto index : indices)
        inliers->indices.push_back(index);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud (cloudRegion);
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (*cloudRegion);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloudRegion;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(const std::unordered_set<int> &indices, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    // typename pcl::ExtractIndices<PointT> extract;
    // typename pcl::PointCloud<PointT>::Ptr obsCloud (new pcl::PointCloud<PointT> ());
    // typename pcl::PointCloud<PointT>::Ptr roadCloud (new pcl::PointCloud<PointT> ());
    // for (int index : inliers->indices)
    //     roadCloud->points.push_back(cloud->points[index]);
    // extract.setInputCloud (cloud);
    // extract.setIndices (inliers);
    // extract.setNegative (true);
    // extract.filter (*obsCloud);
    // std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obsCloud, roadCloud);
    // return segResult;

	typename pcl::PointCloud<PointT>::Ptr  cloudInliers (new pcl::PointCloud<PointT>());
	typename pcl::PointCloud<PointT>::Ptr cloudOutliers (new pcl::PointCloud<PointT>());

	for(int index = 0; index < cloud->points.size(); index++)
	{
		PointT point = cloud->points[index];
		if(indices.count(index))
			cloudInliers->points.push_back(point);
		else
			cloudOutliers->points.push_back(point);
	}
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(cloudOutliers, cloudInliers);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::Ransac(std::unordered_set<int> &indices,typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceTol)
{
	// std::unordered_set<int> inliersResult;
	srand(time(NULL));
	
	// TODO: Fill in this function

	// For max iterations 

	// Randomly sample subset and fit plane

	// Measure distance between every point and fitted line
	// If distance is smaller than threshold count it as inlier

	// Return indicies of inliers from fitted line with most inliers
	while (maxIterations--)
	{
		std::unordered_set<int> inliers;
		while (inliers.size() <3)
			inliers.insert(rand()%(cloud->points.size()));

		float x1, x2, x3, y1, y2, y3, z1, z2, z3;
		auto itr = inliers.begin();
		x1 = cloud->points[*itr].x;
		y1 = cloud->points[*itr].y;
		z1 = cloud->points[*itr].z;
		itr++;
		x2 = cloud->points[*itr].x;
		y2 = cloud->points[*itr].y;
		z2 = cloud->points[*itr].z;
		itr++;
		x3 = cloud->points[*itr].x;
		y3 = cloud->points[*itr].y;
		z3 = cloud->points[*itr].z;

		float A = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1);
		float B = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		float C = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1);
		float D = -(A*x1 + B*y1 + C*z1);

		for (int index = 0; index < cloud->points.size(); index++)
		{
			if (inliers.count(index)>0)
				continue;

			PointT point = cloud->points[index];
			float x4 = point.x;
			float y4 = point.y;
			float z4 = point.z;
			float d = fabs(A*x4 + B*y4 + C*z4 +D)/sqrt(A*A + B*B + C*C);
			if (d <= distanceTol)
				inliers.insert(index);
		}

		if (inliers.size() > indices.size())
		{
			indices = inliers;
		}
	}
    //std::cout << "using own created ransac" << std::endl;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices());
    // //Using the PCL function
    // // TODO:: Fill in this function to find inliers for the cloud.
    // pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients());
    // // create the segmentation object
    // typename pcl::SACSegmentation<PointT> seg;
    // seg.setOptimizeCoefficients(true);
    // seg.setModelType (pcl::SACMODEL_PLANE);
    // seg.setMethodType (pcl::SAC_RANSAC);
    // seg.setMaxIterations (maxIterations);
    // seg.setDistanceThreshold(distanceThreshold);

    // // Segment the largest planar component from the remaining cloud
    // seg.setInputCloud (cloud);
    // seg.segment (*inliers, *coefficients);
    // Using own created ransac segment function
    std::unordered_set<int> indices; 
    Ransac(indices, cloud, maxIterations, distanceThreshold);
    for (auto index : indices)
        inliers->indices.push_back(index);

    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(indices,cloud);
    return segResult;
}

template<typename PointT>
void ProcessPointClouds<PointT>::clusterHelpter(const std::vector<std::vector<float>>& points, int i, std::vector<int> &cluster, std::vector<bool> &processed, KdTree* tree, float distanceTol)
{
	
	processed[i] = 1;
	cluster.push_back(i);
	std::vector<int> nearby = tree->search(points[i],distanceTol);
	for (auto index : nearby)
	{
		if (!processed[index])
		{
			clusterHelpter(points, index, cluster, processed, tree, distanceTol);
		}
	}
	
}

template<typename PointT>
std::vector<std::vector<int>> ProcessPointClouds<PointT>::euclideanCluster(const std::vector<std::vector<float>>& points, KdTree* tree, float distanceTol)
{

	// TODO: Fill out this function to return list of indices for each cluster

	std::vector<std::vector<int>> clusters;
	std::vector<bool> processed(points.size(),false);
	for (int i = 0; i<points.size(); i++)
	{
		if (!processed[i])
		{
			std::vector<int> cluster;
			clusterHelpter(points, i, cluster,processed, tree, distanceTol);
			clusters.push_back(cluster);
		}
	}
 
	return clusters;
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // typename pcl::search::KdTree<PointT>::Ptr tree (new pcl::search::KdTree<PointT>);
    // tree->setInputCloud(cloud);

    // std::vector<pcl::PointIndices> cluster_indices;
    // typename pcl::EuclideanClusterExtraction<PointT> ec;
    // ec.setClusterTolerance (clusterTolerance) ;
    // ec.setMinClusterSize (minSize) ;
    // ec.setMaxClusterSize (maxSize) ;
    // ec.setSearchMethod (tree);
    // ec.setInputCloud (cloud);
    // ec.extract (cluster_indices);

    // for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin(); it != cluster_indices.end(); ++it)
    // {
    //     typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
    //     for (std::vector<int>::const_iterator pit = it->indices.begin();pit != it->indices.end(); ++pit)
    //         cloud_cluster->points.push_back (cloud->points[*pit]);
    //     cloud_cluster->width = cloud_cluster->points.size ();
    //     cloud_cluster->height = 1;
    //     cloud_cluster->is_dense = true;
    //     clusters.push_back(cloud_cluster);
    // }
    // TODO:: Using own created kdtree to realise clustering
    KdTree* kdtree = new KdTree;
    std::vector<std::vector<float>> points;
    for (int i = 0; i<cloud->points.size(); i++)
    {
        std::vector<float> point = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
        points.push_back(point);
    }
    for (int i=0; i<points.size(); i++)
        kdtree->insert(points[i], i);
    std::vector<std::vector<int>> clusters_ids = euclideanCluster(points, kdtree, 1.0);
    for(auto cluster : clusters_ids)
  	{
  		typename pcl::PointCloud<PointT>::Ptr clusterCloud(new pcl::PointCloud<PointT>());
        PointT cluster_point;
  		for(int indice: cluster)
        {
            cluster_point.x =  points[indice][0];
            cluster_point.y =  points[indice][1];
            cluster_point.z =  points[indice][2];
            // cluster_point.intensity =  1.0;
  			clusterCloud->points.push_back(cluster_point);
        }
        clusters.push_back(clusterCloud);
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