#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>


float computeTopPlaneHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cylinderTopCloud,
    const pcl::ModelCoefficients::Ptr& groundPlane)
{
    if (cylinderTopCloud->size() == 0 || groundPlane->values.size() < 4)
        return 0.0f;

    float a = groundPlane->values[0];
    float b = groundPlane->values[1];
    float c = groundPlane->values[2];
    float d = groundPlane->values[3];

    float normalMagnitude = std::sqrt(a*a + b*b + c*c);
    if (normalMagnitude < 1e-6)
        return 0.0f; // Invalid normal

    float totalDistance = 0.0f;
    for (const auto& pt : cylinderTopCloud->points)
    {
        float distance = std::abs(a * pt.x + b * pt.y + c * pt.z + d) / normalMagnitude;
        totalDistance += distance;
    }

    return totalDistance / static_cast<float>(cylinderTopCloud->size());
}

float computeRadiusBasic(pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud) 
{
    if (cloud->size() == 0)
        return 0.0f;

    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, centroid);

    if (centroid.size() < 3)
        return 0.0f;

    float max_dist_sq = 0.0f;
    for (const auto& point : cloud->points) {
        float dx = point.x - centroid[0];
        float dy = point.y - centroid[1];
        float dz = point.z - centroid[2];
        float dist_sq = dx*dx + dy*dy + dz*dz;
        if (dist_sq > max_dist_sq)
            max_dist_sq = dist_sq;
    }

    return std::sqrt(max_dist_sq);
}

float computeRadiusBasicFromConvexHull(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) 
{
    if (cloud->size() == 0)
        return 0.0f;

    pcl::ConvexHull<pcl::PointXYZ> chull;
    pcl::PointCloud<pcl::PointXYZ>::Ptr hull(new pcl::PointCloud<pcl::PointXYZ>);
    chull.setInputCloud(cloud);
    chull.reconstruct(*hull);

    if (hull->points.size() < 2)
        return 0.0f;

    float max_dist_sq = 0.0f;
    for (size_t i = 0; i < hull->size(); ++i) {
        for (size_t j = i + 1; j < hull->size(); ++j) {
            float dx = hull->points[i].x - hull->points[j].x;
            float dy = hull->points[i].y - hull->points[j].y;
            float dz = hull->points[i].z - hull->points[j].z;
            float dist_sq = dx*dx + dy*dy + dz * dz;
            if (dist_sq > max_dist_sq)
                max_dist_sq = dist_sq;
        }
    }

    return std::sqrt(max_dist_sq) / 2.0f; // Approximate radius
}


int main()
{
    // Read the input point cloud
    pcl::PCDReader reader;
    pcl::PLYWriter writer;

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);

    reader.read("../data/recordedPC_000.pcd", *inputCloud);
    std::cout << "Input PointCloud has: " << inputCloud->size() << " data points." << std::endl;


    // Filter the input cloud. TO DO: Statistical outlier removal can be added as well
    pcl::PassThrough<pcl::PointXYZ> passThrough;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Remove too distant points
    passThrough.setInputCloud(inputCloud);
    passThrough.setFilterFieldName("z");
    passThrough.setFilterLimits(0, 1000.0); // Distance set to 1 meter
    passThrough.filter(*filteredCloud);
    std::cout << "Input PointCloud after removing distant points has: " << filteredCloud->size() << " data points." << std::endl;

    // Downsample the cloud
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;

    voxelGrid.setInputCloud(filteredCloud);
    voxelGrid.setLeafSize(4.0f, 4.0f, 4.0f); // CONFIG
    voxelGrid.filter(*filteredCloud);
    std::cout << "Input PointCloud after downsampling has: " << filteredCloud->size() << " data points." << std::endl;
    
    writer.write("recordedPC_01_filtered.ply", *filteredCloud, false);


    // Estimate point normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normalEstimation;
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr kdTree(new pcl::search::KdTree<pcl::PointXYZ>());

    normalEstimation.setSearchMethod(kdTree);
    normalEstimation.setInputCloud(filteredCloud);
    normalEstimation.setKSearch(30);
    normalEstimation.compute(*cloudNormals);


    // Create the segmentation object for the planar model and set its the parameters
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> sacSegmentation; 
    pcl::PointIndices::Ptr planeInliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr planeCoefficients(new pcl::ModelCoefficients);
    
    sacSegmentation.setOptimizeCoefficients(true);
    sacSegmentation.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    sacSegmentation.setNormalDistanceWeight(0.7); // CONFIG
    sacSegmentation.setMethodType(pcl::SAC_RANSAC); // CONFIG
    sacSegmentation.setMaxIterations(100);
    sacSegmentation.setDistanceThreshold(4.0); // CONFIG
    sacSegmentation.setInputCloud(filteredCloud);
    sacSegmentation.setInputNormals(cloudNormals);

    sacSegmentation.segment(*planeInliers, *planeCoefficients);

    // Get the plane normal and normalize it
    Eigen::Vector3f planeNormal(
        planeCoefficients->values[0],
        planeCoefficients->values[1],
        planeCoefficients->values[2]);
    planeNormal.normalize();


    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extractPoints;

    extractPoints.setInputCloud(filteredCloud);
    extractPoints.setIndices(planeInliers);
    extractPoints.setNegative(false);

    // Save the planar inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr planePointCloud(new pcl::PointCloud<pcl::PointXYZ>());

    extractPoints.filter(*planePointCloud);
    std::cout << "PointCloud representing the planar component has: " << planePointCloud->size() << " data points." << std::endl;
    
    writer.write("recordedPC_02_1_segmented_plane.ply", *planePointCloud, false);


    // Extract what is left by removing the planar inliers
    pcl::ExtractIndices<pcl::Normal> extractNormals;

    extractPoints.setNegative(true);
    extractPoints.filter(*filteredCloud);
    extractNormals.setNegative(true);
    extractNormals.setInputCloud(cloudNormals);
    extractNormals.setIndices(planeInliers);
    extractNormals.filter(*cloudNormals);
    writer.write("recordedPC_02_2_segmented_plane_remaining.ply", *filteredCloud, false);
    

    // Create the segmentation object for segmentation of the top of cylinder, set its parameters
    sacSegmentation.setOptimizeCoefficients(true);
    sacSegmentation.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
    sacSegmentation.setNormalDistanceWeight(0.8); // CONFIG
    sacSegmentation.setMethodType(pcl::SAC_RANSAC); // CONFIG
    sacSegmentation.setMaxIterations(10000);
    sacSegmentation.setDistanceThreshold(0.4); // CONFIG
    sacSegmentation.setInputCloud(filteredCloud);
    sacSegmentation.setInputNormals(cloudNormals);
    
    // Set the plane normal as the axis for segmentation
    sacSegmentation.setAxis(planeNormal);

    pcl::PointIndices::Ptr cylinderTopInliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr cylinderTopCoefficients(new pcl::ModelCoefficients);

    sacSegmentation.segment(*cylinderTopInliers, *cylinderTopCoefficients);


    // Save the cylinder top inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinderTopCloud(new pcl::PointCloud<pcl::PointXYZ>());
    extractPoints.setInputCloud(filteredCloud);
    extractPoints.setIndices(cylinderTopInliers);
    extractPoints.setNegative(false);
    extractPoints.filter(*cylinderTopCloud);
    std::cout << "PointCloud representing the top of the cylinder has: " << cylinderTopCloud->size() << " data points." << std::endl;
    
    writer.write("recordedPC_03_segmented_cylinder.ply", *cylinderTopCloud, false);


    // Calculate the height of the cylinder
    float height = computeTopPlaneHeight(cylinderTopCloud, planeCoefficients);
    std::cout << "The height of the given cylinder is: " << height << "mm" << std::endl;


    // Calculate the radius of the cylinder
    float radius = computeRadiusBasic(cylinderTopCloud);
    std::cout << "The radius of the given cylinder is: " << radius << "\n";

    radius = computeRadiusBasicFromConvexHull(cylinderTopCloud);
    std::cout << "The radius of the given cylinder is: " << radius << "\n";
}