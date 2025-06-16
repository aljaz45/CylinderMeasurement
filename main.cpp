#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>
#include <pcl/features/moment_of_inertia_estimation.h>
#include <pcl/surface/mls.h>


int main ()
{
    // Read the input point cloud
    pcl::PCDReader reader;
    pcl::PLYWriter writer;

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud(new pcl::PointCloud<pcl::PointXYZ>);
    reader.read ("../data/recordedPC_000.pcd", *inputCloud);
    std::cout << "Input PointCloud has: " << inputCloud->size() << " data points." << std::endl;


    // Filter the input point cloud using a passthrough filter and downsampling
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    pass.setInputCloud(inputCloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 1200.0); // CONFIG
    pass.filter(*filteredCloud);
    std::cout << "Input PointCloud after filtering has: " << filteredCloud->size() << " data points." << std::endl;

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(filteredCloud);
    vg.setLeafSize(2.0f, 2.0f, 2.0f); // CONFIG
    vg.filter(*filteredCloud);
    writer.write("recordedPC_01_filter_downsample.ply", *filteredCloud, false);
    std::cout << "Input PointCloud after filtering and downsampling has: " << filteredCloud->size() << " data points." << std::endl;


    // Estimate point normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());

    ne.setSearchMethod(tree);
    ne.setInputCloud(filteredCloud);
    ne.setKSearch(50);
    ne.compute(*cloudNormals);


    // Create the segmentation object for the planar model and set all the parameters
    pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg; 
    pcl::PointIndices::Ptr planeInliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr planeCoefficients(new pcl::ModelCoefficients);
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.7); // CONFIG
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(5.0); // CONFIG
    seg.setInputCloud(filteredCloud);
    seg.setInputNormals(cloudNormals);


    // Obtain the plane inliers and coefficients
    seg.segment(*planeInliers, *planeCoefficients);
    std::cout << "Plane coefficients: " << *planeCoefficients << std::endl;
    
    
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(filteredCloud);
    extract.setIndices(planeInliers);
    extract.setNegative(false);
    
    
    // Save the planar inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr planePointCloud(new pcl::PointCloud<pcl::PointXYZ>());
    extract.filter(*planePointCloud);
    std::cout << "PointCloud representing the planar component: " << planePointCloud->size() << " data points." << std::endl;
    writer.write("recordedPC_02_1_segmented_plane.ply", *planePointCloud, false);
    
    
    // Remove the planar inliers, extract the rest
    pcl::ExtractIndices<pcl::Normal> extractNormals;
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals2(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud2(new pcl::PointCloud<pcl::PointXYZ>);

    extract.setNegative(true);
    extract.filter(*filteredCloud2);
    extractNormals.setNegative(true);
    extractNormals.setInputCloud(cloudNormals);
    extractNormals.setIndices(planeInliers);
    extractNormals.filter(*cloudNormals2);
    writer.write("recordedPC_02_2_segmented_plane_remaining.ply", *filteredCloud2, false);
    
    
    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.8); // CONFIG
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(3.0); // CONFIG
    seg.setRadiusLimits(0, 100.0); // CONFIG
    seg.setInputCloud(filteredCloud2);
    seg.setInputNormals(cloudNormals2);
    
    
    // Obtain the cylinder inliers and coefficients
    pcl::PointIndices::Ptr cylinderInliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr cylinderCoefficients(new pcl::ModelCoefficients);
    seg.segment(*cylinderInliers, *cylinderCoefficients);
    std::cout << "Cylinder coefficients: " << *cylinderCoefficients << std::endl;
    
    
    // Save the cylinder inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinderCloud(new pcl::PointCloud<pcl::PointXYZ>());
    extract.setInputCloud(filteredCloud2);
    extract.setIndices(cylinderInliers);
    extract.setNegative(false);
    extract.filter(*cylinderCloud);
    std::cout << "PointCloud representing the cylindrical component: " << cylinderCloud->size() << " data points." << std::endl;
    writer.write("recordedPC_03_segmented_cylinder.ply", *cylinderCloud, false);
    

    //pcl::MomentOfInertiaEstimation<pcl::PointXYZ> feature_extractor;
    //feature_extractor.setInputCloud(cylinderCloud);
    //feature_extractor.compute();

    // Output variables
    //pcl::PointXYZ min_point_AABB, max_point_AABB;
    //pcl::PointXYZ min_point_OBB, max_point_OBB;
    //pcl::PointXYZ position_OBB;
    //Eigen::Matrix3f rotational_matrix_OBB;

    //feature_extractor.getAABB(min_point_AABB, max_point_AABB);
    //feature_extractor.getOBB(min_point_OBB, max_point_OBB, position_OBB, rotational_matrix_OBB);


    // Apply the bounding box to the original input point cloud
    pcl::PointXYZ min_pt, max_pt;
    pcl::getMinMax3D(*cylinderCloud, min_pt, max_pt);

    pcl::CropBox<pcl::PointXYZ> crop_filter;
    crop_filter.setInputCloud(inputCloud);

    // Set the bounding box using min and max from the segmented cloud
    crop_filter.setMin(Eigen::Vector4f(min_pt.x - 5.0f, min_pt.y - 5.0f, min_pt.z - 5.0f, 1.0));
    crop_filter.setMax(Eigen::Vector4f(max_pt.x + 5.0f, max_pt.y + 5.0f, max_pt.z + 5.0f, 1.0));

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cropped(new pcl::PointCloud<pcl::PointXYZ>);
    crop_filter.filter(*cloud_cropped);
    std::cout << "PointCloud representing the cylindrical component of the original input cloud: " << cloud_cropped->size() << " data points." << std::endl;
    writer.write("recordedPC_04_segmented_cylinder_original.ply", *cloud_cropped, false);


    // Use the cropped original cloud to calculate cylinder parameters
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals3(new pcl::PointCloud<pcl::Normal>);

    pcl::PointIndices::Ptr finalCylinderInliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr finalCylinderCoefficients(new pcl::ModelCoefficients);
    
    //vg.setInputCloud(cloud_cropped);
    //vg.setLeafSize(4.0f, 4.0f, 4.0f); // CONFIG
    //vg.filter(*cloud_cropped);
    //std::cout << "PointCloud representing the cylindrical component of the original input cloud: " << cloud_cropped->size() << " data points." << std::endl;
    //writer.write("recordedPC_05_segmented_cylinder_original_downsample.ply", *cloud_cropped, false);

//    pcl::MovingLeastSquares<pcl::PointXYZ, pcl::PointNormal> mls;
//    mls.setInputCloud(cloud_cropped);
//    mls.setPolynomialOrder(2);
//    mls.setSearchMethod(tree);
//    mls.setSearchRadius(3.0);  // Adjust based on point spacing
//    mls.setComputeNormals(true);
//
//    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points(new pcl::PointCloud<pcl::PointNormal>);
//    mls.process(*mls_points);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr mls_xyz(new pcl::PointCloud<pcl::PointXYZ>);
//    pcl::PointCloud<pcl::Normal>::Ptr mls_normals(new pcl::PointCloud<pcl::Normal>);
//
//    for (const auto& pt : mls_points->points) {
//        mls_xyz->points.emplace_back(pt.x, pt.y, pt.z);
//        mls_normals->points.emplace_back(pt.normal_x, pt.normal_y, pt.normal_z);
//    }

    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_cropped);
    ne.setKSearch(30);
    //ne.setRadiusSearch(3.0);
    ne.compute(*cloudNormals3);
    
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_CYLINDER);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.8); // CONFIG
    seg.setMaxIterations(20000);
    seg.setDistanceThreshold(1.0); // CONFIG
    seg.setRadiusLimits(0, 100.0); // CONFIG
    seg.setInputCloud(cloud_cropped);
    seg.setInputNormals(cloudNormals3);

    seg.segment(*finalCylinderInliers, *finalCylinderCoefficients);
    std::cout << "Final Cylinder coefficients: " << *finalCylinderCoefficients << std::endl;


    // Save the final cylinder inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr finalCylinderCloud(new pcl::PointCloud<pcl::PointXYZ>());
    extract.setInputCloud(cloud_cropped);
    extract.setIndices(finalCylinderInliers);
    extract.setNegative(false);
    extract.filter(*finalCylinderCloud);
    std::cout << "PointCloud representing the final cylindrical component: " << finalCylinderCloud->size() << " data points." << std::endl;
    writer.write("recordedPC_06_segmented_cylinder_final.ply", *finalCylinderCloud, false);




//    seg.setOptimizeCoefficients(true);
//    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
//    seg.setNormalDistanceWeight(0.7); // CONFIG
//    seg.setMethodType(pcl::SAC_RANSAC);
//    seg.setMaxIterations(100);
//    seg.setDistanceThreshold(2.0); // CONFIG
//    seg.setInputCloud(cloud_cropped);
//    seg.setInputNormals(cloudNormals3);
//
//    // Obtain the plane inliers and coefficients
//    seg.segment(*planeInliers, *planeCoefficients);
//    std::cout << "Plane coefficients: " << *planeCoefficients << std::endl;
//    
//    // Extract the planar inliers from the input cloud
//    extract.setInputCloud(cloud_cropped);
//    extract.setIndices(planeInliers);
//    extract.setNegative(false);
//    
//    // Save the planar inliers
//    pcl::PointCloud<pcl::PointXYZ>::Ptr planePointCloud2(new pcl::PointCloud<pcl::PointXYZ>());
//    extract.filter(*planePointCloud2);
//    std::cout << "PointCloud representing the final planar component: " << planePointCloud2->size() << " data points." << std::endl;
//    writer.write("recordedPC_segmented_plane_final.ply", *planePointCloud2, false);




    // Compute height
    Eigen::Vector3f axis(cylinderCoefficients->values[3],
    cylinderCoefficients->values[4],
    cylinderCoefficients->values[5]);
    Eigen::Vector3f pt(cylinderCoefficients->values[0],
    cylinderCoefficients->values[1],
    cylinderCoefficients->values[2]);

    float min_proj = std::numeric_limits<float>::max();
    float max_proj = -std::numeric_limits<float>::max();
    for (const auto& p : cylinderCloud->points)
    {
        Eigen::Vector3f vec(p.x - pt[0], p.y - pt[1], p.z - pt[2]);
        float proj = vec.dot(axis);
        if (proj < min_proj) min_proj = proj;
        if (proj > max_proj) max_proj = proj;
    }

    float height = max_proj - min_proj;
    float radius = cylinderCoefficients->values[6];

    std::cout << "Cylinder radius: " << radius << std::endl;
    std::cout << "Cylinder height: " << height << std::endl;


    // Final Compute height
    axis = Eigen::Vector3f(finalCylinderCoefficients->values[3],
    finalCylinderCoefficients->values[4],
    finalCylinderCoefficients->values[5]);
    pt = Eigen::Vector3f(finalCylinderCoefficients->values[0],
    finalCylinderCoefficients->values[1],
    finalCylinderCoefficients->values[2]);

    min_proj = std::numeric_limits<float>::max();
    max_proj = -std::numeric_limits<float>::max();
    for (const auto& p : finalCylinderCloud->points)
    {
        Eigen::Vector3f vec(p.x - pt[0], p.y - pt[1], p.z - pt[2]);
        float proj = vec.dot(axis);
        if (proj < min_proj) min_proj = proj;
        if (proj > max_proj) max_proj = proj;
    }

    height = max_proj - min_proj;
    radius = finalCylinderCoefficients->values[6];

    std::cout << "Final Cylinder radius: " << radius << std::endl;
    std::cout << "Final Cylinder height: " << height << std::endl;

    return (0);
}