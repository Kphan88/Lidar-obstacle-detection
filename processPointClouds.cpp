// PCL lib Functions for processing point clouds 

#include "processPointClouds.h"
#include <unordered_set>
#include "quiz/cluster/kdtree.h"

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
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(
    typename pcl::PointCloud<PointT>::Ptr cloud, 
    float filterRes, 
    Eigen::Vector4f minPoint, 
    Eigen::Vector4f maxPoint)
{

    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // TODO:: Fill in the function to do voxel grid point reduction and region based filtering
    typename pcl::PointCloud<PointT>::Ptr filteredCloud (new pcl::PointCloud<PointT>);

    pcl::VoxelGrid<PointT> vg; 
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes,filterRes,filterRes);
    vg.filter(*filteredCloud);

    typename pcl::PointCloud<PointT>::Ptr cloud_region (new pcl::PointCloud<PointT>); 
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint); 
    region.setMax(maxPoint); 
    region.setInputCloud(filteredCloud); 
    region.filter(*cloud_region);

    // remove roof points 
    
    std::vector<int> roof_indices;

    pcl::CropBox<PointT> roof(true); 
    roof.setMin(Eigen::Vector4f (-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f (2.6, 1.7, 0.4, 1));
    roof.setInputCloud(cloud_region); 
    roof.filter(roof_indices);

    std::cout<<"Debug: cloud size: "<< cloud_region->points.size()<<std::endl; 
    std::cout<<"Debug: roof size: "<< roof_indices.size()<<std::endl; 


    pcl::PointIndices::Ptr roof_inlier {new pcl::PointIndices};
    for (int point :roof_indices)
        roof_inlier->indices.push_back(point); 
    
    pcl::ExtractIndices<PointT> extract; 
    extract.setInputCloud(cloud_region); 
    extract.setIndices(roof_inlier); 
    extract.setNegative(true); 
    extract.filter(*cloud_region);
    


    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return cloud_region;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(
    pcl::PointIndices::Ptr inliers, 
    typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    //std::unordered_set<int> inliers = SegmentPlane(cloud, 100, 0.1);

	typename pcl::PointCloud<PointT>::Ptr groundCloud {new pcl::PointCloud<PointT>()};
	typename pcl::PointCloud<PointT>::Ptr obstacleCloud {new pcl::PointCloud<PointT>()};

   for(int idx :inliers->indices)
   {
        groundCloud->points.push_back(cloud->points[idx]);    
   }
   pcl::ExtractIndices<PointT> extract; 
   extract.setInputCloud(cloud);
   extract.setIndices(inliers); 
   extract.setNegative(true); 
   extract.filter(*obstacleCloud);
   
   extract.setNegative(false); 
   extract.filter(*groundCloud);


    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(groundCloud, obstacleCloud);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(
    typename pcl::PointCloud<PointT>::Ptr cloud, 
    int maxIterations, 
    float distanceThreshold)
{
    // TODO:: Fill in this function to find inliers for the cloud.

    
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    // Use our own fucntion:
    srand(time(NULL));

    pcl::PointIndices::Ptr inliersResult {new pcl::PointIndices()};
     
	while(maxIterations--)
	{
		pcl::PointIndices::Ptr inliers {new pcl::PointIndices()}; 
		while(inliers->indices.size() <3)
		{
			inliers->indices.push_back(rand()%(cloud->points.size()));

		}
        
        
		float x1, y1,z1, x2, y2, z2, x3, y3, z3; 
		auto itr = inliers->indices.begin(); 
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

		float a,b,c,d;

		a = (y2-y1)*(z3-z1)-(z2-z1)*(y3-y1); 
		b = (z2-z1)*(x3-x1)-(x2-x1)*(z3-z1);
		c = (x2-x1)*(y3-y1)-(y2-y1)*(x3-x1); 
		d = -1*(a*x1+b*y1+c*z1); 

		for( int index =0; index < cloud->points.size(); index++)
		{
			if(std::find(inliers->indices.begin(), inliers->indices.end(), index)!= inliers->indices.end())
				continue; 
			PointT point=cloud->points[index];
			float x_t = point.x; 
			float y_t = point.y; 
			float z_t = point.z;

			float distance = fabs(a*x_t+b*y_t+c*z_t+d)/sqrt(a*a+b*b+c*c); 

			if (distance<= distanceThreshold)
				inliers->indices.push_back(index); 

		}

		if (inliers->indices.size() > inliersResult->indices.size())
		{
			inliersResult = inliers; 
		}
	}
	auto endTime = std::chrono::steady_clock::now();
	auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
	std::cout << "Ransac took " << elapsedTime.count() <<" milliseconds" <<std::endl; 
	
     std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliersResult,cloud);
    return segResult;
/*
    //Use PCL libary
    pcl::SACSegmentation<PointT> seg; 
	pcl::PointIndices::Ptr inliers {new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coefficients {new plc::ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC); 
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    // Segment the largest planar component from the point cloud
    seg.setInputCloud(cloud);
    seg.segment(*inliner,*coefficients);
    if (inliers->indices.size() ==0)
    {
        std::cout<< "Could not estimate a planar model from the given dataset."<< std::endl;
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(inliers,cloud);
    return segResult;
    */
}


// define helper function for Clustering: 
template<typename PointT>
void proximityCluster(
	int indice,
	typename pcl::PointCloud<PointT>::Ptr cloud,
	typename pcl::PointCloud<PointT>::Ptr cluster_tmp, 
    float clusterTolerance,
	KdTree* tree, 
	std::vector<bool>& processed)

{
	processed[indice] = true; 
	cluster_tmp->points.push_back(cloud->points[indice]); 

    PointT point = cloud->points[indice];
	std::vector<int> nearests = tree->search({point.x, point.y, point.z}, clusterTolerance);

	//std::cout<<"nearest from indice: "<< nearests.size()<< std::endl;

	for( int id:nearests)
	{
		if (!processed[id])
			proximityCluster(id, cloud, cluster_tmp, clusterTolerance, tree, processed);
	}
	
}
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(
    typename pcl::PointCloud<PointT>::Ptr cloud, 
    float clusterTolerance, 
    int minSize, 
    int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters; 
    
    // create kdTree from the cloud: 
    KdTree* tree = new KdTree; 

    for (int i = 0; i++; i < cloud->points.size())
    {
        PointT point = cloud->points[i]; 
        tree->insert({point.x, point.y, point.z},i); 
    }

    // Perform clustering:
	std::vector<bool> processed(cloud->points.size(), false);

	int i =0; 

	while( i< cloud->points.size())
		{
			if (processed[i])
			{
				i++;
				continue;
			}
				
			typename pcl::PointCloud<PointT>::Ptr cluster_tmp {new pcl::PointCloud<PointT>};

			proximityCluster(i, cloud, cluster_tmp, clusterTolerance, tree, processed);
            if ( (cluster_tmp->points.size() >=minSize) && (cluster_tmp->points.size() <= maxSize) )
			    clusters.push_back(cluster_tmp);
			
			i++;
		}
		
 
	
    // TODO:: Fill in the function to perform euclidean clustering to group detected obstacles
    // Creating the KdTree object for the search method of the extraction
    /*
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance (clusterTolerance); // meter
    ec.setMinClusterSize (minSize);
    ec.setMaxClusterSize (maxSize);
    ec.setSearchMethod (tree);
    ec.setInputCloud (cloud_filtered);
    ec.extract (clusterIndices);

    for(pcl::PointIndices getIndices:clusterIndices)
    {
        //create a temp cluster pointer
        pcl:PointCloud<PointT>::Ptr cloudCluster (new pcl::PointCloud<PointT>);
        //traverse the list of cluster indices
        for (int index : getIndices.indices)
            cloudCluster->points.push_back(cloud->points[index]);
        
        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1; 
        cloudCluster->isDense = true; 

        clusters.push_back(cloudCluster);

    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clusters.size() << " clusters" << std::endl;
    */

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