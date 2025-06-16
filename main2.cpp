#include <depthai/depthai.hpp>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/features/normal_3d.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/common.h>



float computeBoxHeightFromPoints(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr& top_points,
    const pcl::ModelCoefficients::Ptr& ground_plane)
{
    float a = ground_plane->values[0];
    float b = ground_plane->values[1];
    float c = ground_plane->values[2];
    float d = ground_plane->values[3];

    float normal_magnitude = std::sqrt(a*a + b*b + c*c);
    if (normal_magnitude < 1e-6)
        return 0.0f; // Invalid normal

    float total_distance = 0.0f;
    for (const auto& pt : top_points->points)
    {
        float distance = std::abs(a * pt.x + b * pt.y + c * pt.z + d) / normal_magnitude;
        total_distance += distance;
    }

    return total_distance / static_cast<float>(top_points->size());
}

void thefunction(float& radius, float& height, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    // Filter the point cloud: Points that are too distant, and downsample
    pcl::PassThrough<pcl::PointXYZ> pass;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filteredCloud(new pcl::PointCloud<pcl::PointXYZ>);
    
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0, 1000.0);
    pass.filter(*filteredCloud);
    
    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud(filteredCloud);
    vg.setLeafSize(4.0f, 4.0f, 4.0f); // CONFIG
    vg.filter(*filteredCloud);
    
    //sor.setInputCloud(filteredCloud);
    //sor.setMeanK(30);
    //sor.setStddevMulThresh(0.1);
    //sor.filter(*filteredCloud);
    
    
    // Estimate point normals
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
    pcl::PointCloud<pcl::Normal>::Ptr cloudNormals(new pcl::PointCloud<pcl::Normal>);
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    
    ne.setSearchMethod(tree);
    ne.setInputCloud(filteredCloud);
    ne.setKSearch(30);
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
    seg.setDistanceThreshold(4.0); // CONFIG
    seg.setInputCloud(filteredCloud);
    seg.setInputNormals(cloudNormals);
    
    
    // Obtain the plane inliers and coefficients
    seg.segment(*planeInliers, *planeCoefficients);
    
    // Extract the planar inliers from the input cloud
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(filteredCloud);
    extract.setIndices(planeInliers);
    extract.setNegative(false);
    
    
    // Save the planar inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr planePointCloud(new pcl::PointCloud<pcl::PointXYZ>());
    extract.filter(*planePointCloud);
    
    
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
    
    if (planeCoefficients->values.size() < 3)
    return;
    
    // Create the segmentation object for cylinder segmentation and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PARALLEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setNormalDistanceWeight(0.8); // CONFIG
    seg.setMaxIterations(10000);
    seg.setDistanceThreshold(0.4); // CONFIG
    seg.setRadiusLimits(0, 100.0); // CONFIG
    seg.setInputCloud(filteredCloud2);
    seg.setInputNormals(cloudNormals2);
            std::cout << "ss123ss" << std::endl;
    Eigen::Vector3f normal(planeCoefficients->values[0],
        planeCoefficients->values[1],
        planeCoefficients->values[2]);
        
                std::cout << "sss23s" << std::endl;
    // Normalize the normal vector
    normal.normalize();
        
        std::cout << "ssss" << std::endl;
    // Set this normal as the axis
    seg.setAxis(normal);
    //seg.setEpsAngle(pcl::deg2rad(5.0));  // Allow planes within ±5° of this direction
    
    
    // Obtain the cylinder inliers and coefficients
    pcl::PointIndices::Ptr cylinderInliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr cylinderCoefficients(new pcl::ModelCoefficients);
    seg.segment(*cylinderInliers, *cylinderCoefficients);


    
    // Save the cylinder inliers
    pcl::PointCloud<pcl::PointXYZ>::Ptr cylinderCloud(new pcl::PointCloud<pcl::PointXYZ>());
    extract.setInputCloud(filteredCloud2);
    extract.setIndices(cylinderInliers);
    extract.setNegative(false);
    extract.filter(*cylinderCloud);

    height = computeBoxHeightFromPoints(cylinderCloud, planeCoefficients);
}



int main()
{
    // Create pipeline
    dai::Pipeline pipeline;

    auto left = pipeline.create<dai::node::Camera>();
    auto right = pipeline.create<dai::node::Camera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto pointcloud = pipeline.create<dai::node::PointCloud>();
    auto color = pipeline.create<dai::node::Camera>()->build();
    std::shared_ptr<dai::node::ImageAlign> align;

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


    // Main loop
    while(true) 
    {
        auto videoIn = videoQueue->get<dai::ImgFrame>();
        if(videoIn == nullptr) 
        continue;
        
        if(cv::waitKey(1) == 'q')
        break;
        
        auto pcl = queue->get<dai::PointCloudData>();
        if (pcl == nullptr)
        continue;
        
        // Fill the cloud ...   TO DO: Probably a better way to do this ...
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        cloud->width = pcl->getWidth();
        cloud->height = pcl->getHeight();
        cloud->points.reserve(pcl->getPoints().size());
        for (const auto& point : pcl->getPoints()) 
        cloud->points.emplace_back(point.x, point.y, point.z);
        
        float radius = 0.0f;
        float height = 0.0f;
        thefunction(radius, height, cloud);
        
        
        // Convert variables to string
        std::stringstream ss;
        ss << "Radius: " << radius << ",   Height: " << height;
        std::string text = ss.str();
        
        // Text properties
        cv::Point position(30, 200);
        int fontFace = cv::FONT_HERSHEY_SIMPLEX;
        double fontScale = 0.8;
        cv::Scalar fontColor(255, 255, 255); // White
        int thickness = 2;
        
        // Draw text
        cv::Mat test = videoIn->getCvFrame();
        cv::putText(test, text, position, fontFace, fontScale, fontColor, thickness);

        cv::imshow("video", test);
    }
}