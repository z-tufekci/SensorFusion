// PCL lib Functions for processing point clouds

#include "processPointClouds.h"
#include <unordered_set>

//constructor:
template <typename PointT>
ProcessPointClouds<PointT>::ProcessPointClouds() {}

//de-constructor:
template <typename PointT>
ProcessPointClouds<PointT>::~ProcessPointClouds() {}

template <typename PointT>
void ProcessPointClouds<PointT>::numPoints(typename pcl::PointCloud<PointT>::Ptr cloud)
{
    std::cout << cloud->points.size() << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::FilterCloud(typename pcl::PointCloud<PointT>::Ptr cloud, float filterRes, Eigen::Vector4f minPoint, Eigen::Vector4f maxPoint)
{
    auto startTime = std::chrono::steady_clock::now();

    pcl::VoxelGrid<PointT> voxgrid;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(new pcl::PointCloud<PointT>);
    voxgrid.setInputCloud(cloud);
    voxgrid.setLeafSize(filterRes, filterRes, filterRes);
    voxgrid.filter(*cloudFiltered);

    
    typename pcl::PointCloud<PointT>::Ptr cloudLast(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> cropbox(true);
    cropbox.setMin(minPoint);
    cropbox.setMax(maxPoint);
    cropbox.setInputCloud(cloudFiltered);
    cropbox.filter(*cloudLast);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.5, 1.7, -0.4, 1));
    roof.setInputCloud(cloudLast);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers{new pcl::PointIndices};
    for (int point : indices)
    {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudLast);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudLast);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;
    std::cout << "After filtering " << (cloudLast->width) * (cloudLast->height) << "  " << pcl::getFieldsList(*cloudLast) << std::endl;

    return cloudLast;
}

/*
template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
  // TODO: Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obs{ new pcl::PointCloud<PointT>() }; 
    typename pcl::PointCloud<PointT>::Ptr plane{ new pcl::PointCloud<PointT>() };

    for(int index : inliers -> indices )   
        plane -> points.push_back( cloud -> points[index]);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obs);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obs, plane);
    return segResult;
}
*/

template <typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    auto startTime = std::chrono::steady_clock::now();
    //RANSAC
    std::unordered_set<int> inliersResult;
    srand(time(NULL));

    while (maxIterations--)
    {
        std::unordered_set<int> list;
        while (list.size() < 3)
            list.insert(rand() % (cloud->points.size()));
        float x1, y1, z1, x2, y2, z2, x3, y3, z3;
        auto itr = list.begin();

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

        float i, j, k, d;
        i = (y2 - y1) * (z3 - z1) - (z2 - z1) * (y3 - y1);
        j = (z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1);
        k = (x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1);
        d = -1 * (i * x1 + j * y1 + k * z1);

        for (int index = 0; index < cloud->points.size(); index++)
        {
            if (list.count(index) > 0)
            {
                continue;
            }
            PointT point = cloud->points[index];

            float dist = fabs(i * (point.x) + j * (point.y) + k * (point.z) + d) / sqrt(i * i + j * j + k * k);

            if (dist <= distanceThreshold)
                list.insert(index);
        }
        if (list.size() > inliersResult.size())
            inliersResult = list;
    }
    //AFTER RANSAC INLIERS AND OUTLIERS
    typename pcl::PointCloud<PointT>::Ptr cloudInliers(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr cloudOutliers(new pcl::PointCloud<PointT>());

    for (int index = 0; index < cloud->points.size(); index++)
    {
        PointT point = cloud->points[index];
        if (inliersResult.count(index))
            cloudInliers->points.push_back(point);
        else
            cloudOutliers->points.push_back(point);
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = std::make_pair(cloudOutliers, cloudInliers);
    return segResult;

    /*
    pcl::SACSegmentation<PointT> seg;
    pcl::PointIndices::Ptr inliers { new pcl::PointIndices};
    pcl::ModelCoefficients::Ptr coef { new pcl::ModelCoefficients};

    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(maxIterations);
    seg.setDistanceThreshold(distanceThreshold);

    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coef);
    */
}

template <typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(const typename pcl::PointCloud<PointT>::Ptr &cloud, float clusterTolerance, int minSize, int maxSize)
{
    auto startTime = std::chrono::steady_clock::now();
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clustersResulted;

    //cout << typeid(cloud->points).name() <<std::endl;
    int psize = (cloud->points).size();
   
    KdTree* tree = new KdTree;
    for (int i=0; i< psize ; i++) {
        std::vector<float> vecpoint = {cloud->points[i].x, cloud->points[i].y,cloud->points[i].z};
      	tree->insert(vecpoint,i); 
    }
    std::vector<std::vector<int>> clusters;

	std::vector<bool> processed(psize, false); 

	int i = 0;
	for(PointT point : (cloud->points)){
		// new cluster
		if(!processed[i]){ //if this point has not visited
			//create cluster
			std::vector<int> cluster;
			//proximity 
			clusterHelp(i, cloud, cluster, processed, tree, clusterTolerance);
			// add cluster to clusters
			clusters.push_back(cluster);
		}
        i++;
	}

  	for(std::vector<int> cluster : clusters)
  	{
        if(cluster.size() < minSize || cluster.size() > maxSize)
            continue;

  		pcl::PointCloud<pcl::PointXYZ>::Ptr clusterCloud(new pcl::PointCloud<pcl::PointXYZ>());
  		for(int indice: cluster)
  			clusterCloud->points.push_back(cloud->points[indice]);
  		
        clusterCloud->width = clusterCloud->points.size();
        clusterCloud->height = 1;
        clusterCloud->is_dense = true;

        clustersResulted.push_back(clusterCloud);
  	}

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "clustering took " << elapsedTime.count() << " milliseconds and found " << clustersResulted.size() << " clusters" << std::endl;

    return clustersResulted;
    /*
    typename pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(clusterTolerance);
    ec.setMinClusterSize(minSize);
    ec.setMaxClusterSize(maxSize);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    for (pcl::PointIndices getindex : cluster_indices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloudCluster(new pcl::PointCloud<PointT>);

        for (int index : getindex.indices)
            cloudCluster->points.push_back(cloud->points[index]);

        cloudCluster->width = cloudCluster->points.size();
        cloudCluster->height = 1;
        cloudCluster->is_dense = true;

        clusters.push_back(cloudCluster);
    }
    */
}

template <typename PointT>
void ProcessPointClouds<PointT>::clusterHelp(int index,const typename pcl::PointCloud<PointT>::Ptr &pointcloud, std::vector<int>& cluster,std::vector<bool>& processed, KdTree* tree, float distanceTol)
{
	//if not visited
	if(!processed[index]){
		// add to visited
		processed[index] = true;
		// add point to cluster
		cluster.push_back(index);
		// find nearby points 
		// tree search
        std::vector<float> point = { pointcloud-> points[index].x, pointcloud->points[index].y, pointcloud->points[index].z};
		std::vector<int> ids = tree -> search(point, distanceTol);

		//iterate through nearby points
		for(int id : ids)
			if(!processed[id])
				clusterHelp(id, pointcloud, cluster, processed, tree, distanceTol);
	}
}

template <typename PointT>
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

template <typename PointT>
void ProcessPointClouds<PointT>::savePcd(typename pcl::PointCloud<PointT>::Ptr cloud, std::string file)
{
    pcl::io::savePCDFileASCII(file, *cloud);
    std::cerr << "Saved " << cloud->points.size() << " data points to " + file << std::endl;
}

template <typename PointT>
typename pcl::PointCloud<PointT>::Ptr ProcessPointClouds<PointT>::loadPcd(std::string file)
{

    typename pcl::PointCloud<PointT>::Ptr cloud(new pcl::PointCloud<PointT>);

    if (pcl::io::loadPCDFile<PointT>(file, *cloud) == -1) //* load the file
    {
        PCL_ERROR("Couldn't read file \n");
    }
    std::cerr << "Loaded " << cloud->points.size() << " data points from " + file << std::endl;

    return cloud;
}

template <typename PointT>
std::vector<boost::filesystem::path> ProcessPointClouds<PointT>::streamPcd(std::string dataPath)
{
    std::vector<boost::filesystem::path> paths(boost::filesystem::directory_iterator{dataPath}, boost::filesystem::directory_iterator{});
    // sort files in accending order so playback is chronological
    sort(paths.begin(), paths.end());

    return paths;
}