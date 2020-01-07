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

    // Fill in the function to do voxel grid point reduction and region based filtering
    pcl::VoxelGrid<PointT> voxel_grid;
    typename pcl::PointCloud<PointT>::Ptr reduced_cloud (new pcl::PointCloud<PointT>);
    voxel_grid.setInputCloud(cloud);
    voxel_grid.setLeafSize(filterRes, filterRes, filterRes);
    voxel_grid.filter(*reduced_cloud);

    // Remove points outside region of interest
    typename pcl::PointCloud<PointT>::Ptr region_cloud(new pcl::PointCloud<PointT>);
    pcl::CropBox<PointT> region_of_interest(true);
    region_of_interest.setInputCloud(reduced_cloud);
    region_of_interest.setMin(minPoint);
    region_of_interest.setMax(maxPoint);
    region_of_interest.filter(*region_cloud);

    // Remove points inside roof box.
    std::vector<int> indices;
    pcl::CropBox<PointT> roof(true);
    roof.setMin(Eigen::Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Eigen::Vector4f(2.6, 1.7, -.4, 1));
    roof.setInputCloud(region_cloud);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (int point : indices) {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(region_cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*region_cloud);

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "filtering took " << elapsedTime.count() << " milliseconds" << std::endl;

    return region_cloud;

}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SeparateClouds(pcl::PointIndices::Ptr inliers, typename pcl::PointCloud<PointT>::Ptr cloud) 
{
    // Create two new point clouds, one cloud with obstacles and other with segmented plane
    typename pcl::PointCloud<PointT>::Ptr obstacles(new pcl::PointCloud<PointT>());
    typename pcl::PointCloud<PointT>::Ptr plane(new pcl::PointCloud<PointT>());

    for (int index : inliers->indices) {
        plane->points.push_back(cloud->points[index]);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*obstacles);

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult(obstacles, plane);
    return segResult;
}


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::SegmentPlane(typename pcl::PointCloud<PointT>::Ptr cloud, int maxIterations, float distanceThreshold)
{
    // Time segmentation process
    auto startTime = std::chrono::steady_clock::now();

    pcl::PointIndices::Ptr bestInliers (new pcl::PointIndices());

    while (maxIterations--) {
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices());

        // Get 3 different random indices.
        for (int i = 0; i < cloud->points.size(); i++) {
            inliers->indices.push_back(i);
        }

        std::random_shuffle(inliers->indices.begin(), inliers->indices.end());
        inliers->indices.resize(3);

        // Indices 0, 1, 2 is the value of the respective points.
        // Index 3 is the difference between 1 and 0.
        // Index 4 is the difference between 2 and 0.
        float x[5];
        float y[5];
        float z[5];

        // Fill data.
        auto itr = inliers->indices.begin();
        x[0] = cloud->points[*itr].x;
        y[0] = cloud->points[*itr].y;
        z[0] = cloud->points[*itr].z;
        ++itr;
        x[1] = cloud->points[*itr].x;
        y[1] = cloud->points[*itr].y;
        z[1] = cloud->points[*itr].z;
        ++itr;
        x[2] = cloud->points[*itr].x;
        y[2] = cloud->points[*itr].y;
        z[2] = cloud->points[*itr].z;

        x[3] = x[1] - x[0];
        x[4] = x[2] - x[0];
        y[3] = y[1] - y[0];
        y[4] = y[2] - y[0];
        z[3] = z[1] - z[0];
        z[4] = z[2] - z[0];

        // Determine plane.
        float a = (y[3] * z[4]) - (z[3] * y[4]);
        float b = (z[3] * x[4]) - (x[3] * z[4]);
        float c = (x[3] * y[4]) - (y[3] * x[4]);
        float d = -((a * x[0]) + (b * y[0]) + (c * z[0]));

        for (int index = 0; index < cloud->points.size(); index++) {
            // Skip if index is part of plane.
            if ((inliers->indices[0] == index) || (inliers->indices[1] == index) || (inliers->indices[2] == index)) {
                continue;
            }

            PointT point = cloud->points[index];

            float distance = fabs((a * point.x) + (b * point.y) + (c * point.z) + d) / sqrt((a * a) + (b * b) + (c * c));

            if (distance <= distanceThreshold) {
                inliers->indices.push_back(index);
            }
        }

        if (inliers->indices.size() > bestInliers->indices.size()) {
            bestInliers = inliers;
        }
    }

    auto endTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(endTime - startTime);
    std::cout << "plane segmentation took " << elapsedTime.count() << " milliseconds" << std::endl;

    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segResult = SeparateClouds(bestInliers,cloud);
    return segResult;
}

template<typename PointT>
struct Node {
    PointT point;
    int id;
    Node* left;
    Node* right;

    Node(PointT setPoint, int setId)
        : point(setPoint), id(setId), left(NULL), right(NULL)
    {}
};

template<typename PointT>
struct KdTree {
    Node<PointT>* root;

    KdTree()
        : root(NULL)
    {}

    void insert(PointT point, int id) {
        insert_helper(&root, id, 0, point);
    }

    void insert_helper(Node<PointT>** node, int id, int depth, PointT point) {
        if (*node == NULL) {
            *node = new Node<PointT>(point, id);
        } else {
            int d = depth % 3;
            float node_d;
            float point_d;

            if (d == 0) {
                node_d = (*node)->point.x;
                point_d = point.x;
            } else if (d == 1) {
                node_d = (*node)->point.y;
                point_d = point.y;
            } else {
                node_d = (*node)->point.z;
                point_d = point.z;
            }

            if (point_d < node_d) {
                insert_helper(&(*node)->left, id, depth + 1, point);
            } else {
                insert_helper(&(*node)->right, id, depth + 1, point);
            }
        }
    }

    std::vector<int> search(PointT target, float tolerance) {
        std::vector<int> ids;
        search_helper(target, tolerance, root, 0, ids);
        return ids;
    }

    void search_helper(PointT target, float tolerance, Node<PointT>* node, int depth, std::vector<int>& ids) {
        if (node != NULL) {
            if (node->point.x >= target.x - tolerance
                    && node->point.x <= target.x + tolerance
                    && node->point.y >= target.y - tolerance
                    && node->point.y <= target.y + tolerance
                    && node->point.z >= target.z - tolerance
                    && node->point.z <= target.z + tolerance) {
                float distance_x = node->point.x - target.x;
                float distance_y = node->point.y - target.y;
                float distance_z = node->point.z - target.z;
                float distance = sqrt((distance_x * distance_x) + (distance_y * distance_y) + (distance_z * distance_z));

                if (distance < tolerance) {
                    ids.push_back(node->id);
                }
            }

            int d = depth % 3;
            float node_d;
            float target_d;

            if (d == 0) {
                node_d = node->point.x;
                target_d = target.x;
            } else if (d == 1) {
                node_d = node->point.y;
                target_d = target.y;
            } else {
                node_d = node->point.z;
                target_d = target.z;
            }

            if (node_d > target_d - tolerance) {
                search_helper(target, tolerance, node->left, depth + 1, ids);
            }

            if (node_d < target_d + tolerance) {
                search_helper(target, tolerance, node->right, depth + 1, ids);
            }
        }
    }
};


template<typename PointT>
void proximity(typename pcl::PointCloud<PointT>::Ptr cloud, int index, typename pcl::PointCloud<PointT>::Ptr cluster, std::vector<bool>& hasBeenProcessed, KdTree<PointT>* tree, float clusterTolerance) {
    hasBeenProcessed[index] = true;
    cluster->points.push_back(cloud->points[index]);
    std::vector<int> nearbyIndices = tree->search(cloud->points[index], clusterTolerance);

    for (int nearby_index : nearbyIndices) {
        if (!hasBeenProcessed[nearby_index]) {
            proximity(cloud, nearby_index, cluster, hasBeenProcessed, tree, clusterTolerance);
        }
    }
}

template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> ProcessPointClouds<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr cloud, float clusterTolerance, int minSize, int maxSize)
{

    // Time clustering process
    auto startTime = std::chrono::steady_clock::now();

    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;
    std::vector<bool> hasBeenProcessed(cloud->points.size(), false);
    KdTree<PointT>* tree = new KdTree<PointT>;

    for (int i = 0; i < cloud->points.size(); i++) {
        tree->insert(cloud->points[i], i);
    }

    for (int i = 0; i < cloud->points.size(); i++) {
        if (!hasBeenProcessed[i]) {
            typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>());
            proximity(cloud, i, cluster, hasBeenProcessed, tree, clusterTolerance);

            int size = cluster->points.size();

            if ((size >= minSize) && (size <= maxSize)) {
                clusters.push_back(cluster);
            }
        }
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
