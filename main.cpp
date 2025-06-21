#include <depthai/depthai.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>


float computeHeight(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cylinderTopCloud,
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
        // ax + by + cz + d
        float distance = std::abs(a * pt.x + b * pt.y + c * pt.z + d) / normalMagnitude;
        totalDistance += distance;
    }

    return totalDistance / static_cast<float>(cylinderTopCloud->size());
}

float computeRadius(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cylinderTopHullCloud) 
{
    if (cylinderTopHullCloud->points.size() < 2)
        return 0.0f;

    // Find the longest distance between the points - the diameter
    float maxDistance = 0.0f;
    for (size_t i = 0; i < cylinderTopHullCloud->size(); ++i) 
    {
        for (size_t j = i + 1; j < cylinderTopHullCloud->size(); ++j) 
        {
            float dx = cylinderTopHullCloud->points[i].x - cylinderTopHullCloud->points[j].x;
            float dy = cylinderTopHullCloud->points[i].y - cylinderTopHullCloud->points[j].y;
            float dz = cylinderTopHullCloud->points[i].z - cylinderTopHullCloud->points[j].z;
            float distance = dx * dx + dy * dy + dz * dz;
            if (distance > maxDistance)
                maxDistance = distance;
        }
    }

    // Return the radius
    return std::sqrt(maxDistance) / 2.0f;
}

void computeCylinderDimensions(float& radius, float& height, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // Filter the input cloud. TO DO: Statistical outlier removal can be added as well
    pcl::PassThrough<pcl::PointXYZ> passThrough;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);

    // Remove too distant points
    passThrough.setInputCloud(cloud);
    passThrough.setFilterFieldName("z");
    passThrough.setFilterLimits(0, 1000.0); // Distance set to 1 meter
    passThrough.filter(*filteredCloud);

    // Downsample the cloud
    pcl::VoxelGrid<pcl::PointXYZ> voxelGrid;

    voxelGrid.setInputCloud(filteredCloud);
    voxelGrid.setLeafSize(4.0f, 4.0f, 4.0f); // CONFIG
    voxelGrid.filter(*filteredCloud);


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
    sacSegmentation.setDistanceThreshold(3.5); // CONFIG
    sacSegmentation.setInputCloud(filteredCloud);
    sacSegmentation.setInputNormals(cloudNormals);

    sacSegmentation.segment(*planeInliers, *planeCoefficients);

    // Return in case of an invalid plane coefficients
    if (planeCoefficients->values.size() < 4)
        return;

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


    // Extract what is left by removing the planar inliers
    pcl::ExtractIndices<pcl::Normal> extractNormals;

    extractPoints.setNegative(true);
    extractPoints.filter(*filteredCloud);
    extractNormals.setNegative(true);
    extractNormals.setInputCloud(cloudNormals);
    extractNormals.setIndices(planeInliers);
    extractNormals.filter(*cloudNormals);
    

    // Create the segmentation object for segmentation of the top of cylinder, set its parameters
    sacSegmentation.setOptimizeCoefficients(true);
    sacSegmentation.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
    sacSegmentation.setNormalDistanceWeight(0.8); // CONFIG
    sacSegmentation.setMethodType(pcl::SAC_RANSAC); // CONFIG
    sacSegmentation.setMaxIterations(3000);
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


    // Get the convex hull of the cylinder top and reconstruct it to a point cloud
    pcl::ConvexHull<pcl::PointXYZ> convexHull;
    pcl::PointCloud<pcl::PointXYZ>::Ptr hullCloud(new pcl::PointCloud<pcl::PointXYZ>);
    convexHull.setInputCloud(cylinderTopCloud);
    convexHull.reconstruct(*hullCloud);


    // Calculate the height of the cylinder
    height = computeHeight(cylinderTopCloud, planeCoefficients);

    // Calculate the radius of the cylinder
    radius = computeRadius(hullCloud);
}

float updateMedianFilter(std::deque<float>& measuredValues, float measurement, size_t maxMeasurements)
{
    measuredValues.push_back(measurement);
    if (measuredValues.size() > maxMeasurements)
        measuredValues.pop_front();

    std::vector<double> sortedValues(measuredValues.begin(), measuredValues.end());
    std::sort(sortedValues.begin(), sortedValues.end());
    size_t medianIndex = sortedValues.size() / 2;

    // Average the middle elements in case of even number of measurements
    if (sortedValues.size() % 2 == 0)
        return (sortedValues[medianIndex - 1] + sortedValues[medianIndex]) / 2.0;
    else
        return sortedValues[medianIndex];
}


int main()
{
    // Create pipeline
    dai::Pipeline pipeline;

    // Create nodes
    auto left = pipeline.create<dai::node::Camera>();
    auto right = pipeline.create<dai::node::Camera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto pointcloud = pipeline.create<dai::node::PointCloud>();
    auto color = pipeline.create<dai::node::Camera>()->build();
    std::shared_ptr<dai::node::ImageAlign> align;

    // Build and link nodes
    left->build(dai::CameraBoardSocket::CAM_B);
    right->build(dai::CameraBoardSocket::CAM_C);
    stereo->setSubpixel(true);
    stereo->setExtendedDisparity(false);
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::HIGH_DETAIL);
    stereo->setLeftRightCheck(true);
    stereo->setRectifyEdgeFillColor(0);  // black, to better see the cutout
    stereo->enableDistortionCorrection(true);
    stereo->initialConfig->setLeftRightCheckThreshold(10);

    left->requestOutput(std::pair<int, int>(640, 400))->link(stereo->left);
    right->requestOutput(std::pair<int, int>(640, 400))->link(stereo->right);


    auto platform = pipeline.getDefaultDevice()->getPlatform();
    dai::Node::Output* colorOutput;
    if(platform == dai::Platform::RVC4) {
        colorOutput = color->requestOutput(std::pair<int, int>(640, 400), dai::ImgFrame::Type::RGB888i);
        align = pipeline.create<dai::node::ImageAlign>();
        stereo->depth.link(align->input);
        colorOutput->link(align->inputAlignTo);
    } else {
        colorOutput = color->requestOutput(std::pair<int, int>(640, 400), dai::ImgFrame::Type::RGB888i, dai::ImgResizeMode::CROP, 30, true);
        colorOutput->link(stereo->inputAlignTo);
    }

    // Video
    auto videoQueue = colorOutput->createOutputQueue();

    // Point cloud
    stereo->depth.link(pointcloud->inputDepth);
    auto queue = pointcloud->outputPointCloud.createOutputQueue();

    auto device = pipeline.getDefaultDevice();
    device->setIrLaserDotProjectorIntensity(0.7);

    // Start pipeline
    pipeline.start();


    double fps = 0;
    int frameCount = 0;
    auto startTime = std::chrono::high_resolution_clock::now();

    size_t numberOfValues = 10;
    std::deque<float> measuredValuesHeight;
    std::deque<float> measuredValuesRadius;


    while(true) 
    {
        if(cv::waitKey(1) == 'q')
            break;

        auto videoIn = videoQueue->get<dai::ImgFrame>();
        if(videoIn == nullptr) 
            continue;

        auto pointCloudData = queue->get<dai::PointCloudData>();
        if (pointCloudData == nullptr)
            continue;

        // Fill the cloud. TO DO: Probably a better way to do this ...
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->width = pointCloudData->getWidth();
        cloud->height = pointCloudData->getHeight();
        cloud->points.reserve(pointCloudData->getPoints().size());
        for (const auto& point : pointCloudData->getPoints()) 
            cloud->points.emplace_back(point.x, point.y, point.z);
        
        // Get the cylinder dimensions
        float height = 0.0f;
        float radius = 0.0f;
        computeCylinderDimensions(radius, height, cloud);
        
        height = updateMedianFilter(measuredValuesHeight, height, numberOfValues);
        radius = updateMedianFilter(measuredValuesRadius, radius, numberOfValues);
            
            
        // Calculate FPS (every 3 frames)
        frameCount++;
        if (frameCount == 3) 
        {
            auto endTime = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsedTime = endTime - startTime;
            fps = frameCount / elapsedTime.count();

            frameCount = 0;
            startTime = endTime;
        }

        
        // Display the FPS and measurement results
        std::stringstream stringStream;
        stringStream << "Radius: " << std::roundf(radius) << " mm   Height: " << std::roundf(height) << " mm";
        std::string measurementText = stringStream.str();

        stringStream.str("");
        stringStream << "Current FPS: " << fps;
        std::string fpsText = stringStream.str();
        
        cv::Mat currentFrame = videoIn->getCvFrame();
        cv::putText(currentFrame, measurementText, cv::Point(100, 350), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);
        cv::putText(currentFrame, fpsText, cv::Point(0, 24), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 255, 255), 2);

        cv::imshow("video", currentFrame);
    }
}