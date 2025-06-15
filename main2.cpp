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


void thefunction(float& radius, float& height, pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
  // All the objects needed
  pcl::PassThrough<pcl::PointXYZ> pass;
  pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
  pcl::SACSegmentationFromNormals<pcl::PointXYZ, pcl::Normal> seg; 
  pcl::ExtractIndices<pcl::PointXYZ> extract;
  pcl::ExtractIndices<pcl::Normal> extract_normals;
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ> ());

  // Datasets
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals (new pcl::PointCloud<pcl::Normal>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered2 (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2 (new pcl::PointCloud<pcl::Normal>);
  pcl::ModelCoefficients::Ptr coefficients_plane (new pcl::ModelCoefficients), coefficients_cylinder (new pcl::ModelCoefficients);
  pcl::PointIndices::Ptr inliers_plane (new pcl::PointIndices), inliers_cylinder (new pcl::PointIndices);

  // Read in the cloud data
  std::cerr << "PointCloud has: " << cloud->size () << " data points." << std::endl;

  // Build a passthrough filter to remove spurious NaNs and scene background
  pass.setInputCloud (cloud);
  pass.setFilterFieldName ("z");
  pass.setFilterLimits (0, 1200.0);
  pass.filter (*cloud_filtered);
  std::cerr << "PointCloud after filtering has: " << cloud_filtered->size () << " data points." << std::endl;
  
  pcl::VoxelGrid<pcl::PointXYZ> vg;
  vg.setInputCloud(cloud_filtered);
  vg.setLeafSize(2.0f, 2.0f, 2.0f);
  vg.filter(*cloud_filtered);
  std::cerr << "PointCloud after downsampling has: " << cloud_filtered->size () << " data points." << std::endl;

  // Estimate point normals
  ne.setSearchMethod (tree);
  ne.setInputCloud (cloud_filtered);
  ne.setKSearch (50);
  ne.compute (*cloud_normals);

  // Create the segmentation object for the planar model and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_NORMAL_PLANE);
  seg.setNormalDistanceWeight (0.1);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setMaxIterations (100);
  seg.setDistanceThreshold (4.0);
  seg.setInputCloud (cloud_filtered);
  seg.setInputNormals (cloud_normals);
  // Obtain the plane inliers and coefficients
  seg.segment (*inliers_plane, *coefficients_plane);
  std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

  // Extract the planar inliers from the input cloud
  extract.setInputCloud (cloud_filtered);
  extract.setIndices (inliers_plane);
  extract.setNegative (false);

  // Write the planar inliers to disk
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
  extract.filter (*cloud_plane);
  std::cerr << "PointCloud representing the planar component: " << cloud_plane->size () << " data points." << std::endl;

  // Remove the planar inliers, extract the rest
  extract.setNegative (true);
  extract.filter (*cloud_filtered2);
  extract_normals.setNegative (true);
  extract_normals.setInputCloud (cloud_normals);
  extract_normals.setIndices (inliers_plane);
  extract_normals.filter (*cloud_normals2);

  // Create the segmentation object for cylinder segmentation and set all the parameters
  seg.setOptimizeCoefficients (true);
  seg.setModelType (pcl::SACMODEL_CYLINDER);
  seg.setMethodType (pcl::SAC_RANSAC);
  seg.setNormalDistanceWeight (0.1);
  seg.setMaxIterations (10000);
  seg.setDistanceThreshold (4.0);
  seg.setRadiusLimits (0, 100.0);
  seg.setInputCloud (cloud_filtered2);
  seg.setInputNormals (cloud_normals2);

  // Obtain the cylinder inliers and coefficients
  seg.segment (*inliers_cylinder, *coefficients_cylinder);
  std::cerr << "Cylinder coefficients: " << *coefficients_cylinder << std::endl;

  // Write the cylinder inliers to disk
  extract.setInputCloud (cloud_filtered2);
  extract.setIndices (inliers_cylinder);
  extract.setNegative (false);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cylinder (new pcl::PointCloud<pcl::PointXYZ> ());
  extract.filter (*cloud_cylinder);
  if (cloud_cylinder->points.empty ()) 
    std::cerr << "Can't find the cylindrical component." << std::endl;
  else
  {
	  std::cerr << "PointCloud representing the cylindrical component: " << cloud_cylinder->size () << " data points." << std::endl;
  }

  // Compute height
  Eigen::Vector3f axis(coefficients_cylinder->values[3],
                        coefficients_cylinder->values[4],
                        coefficients_cylinder->values[5]);
  Eigen::Vector3f pt(coefficients_cylinder->values[0],
                      coefficients_cylinder->values[1],
                      coefficients_cylinder->values[2]);

  float min_proj = std::numeric_limits<float>::max();
  float max_proj = -std::numeric_limits<float>::max();
  for (const auto& p : cloud_cylinder->points)
  {
      Eigen::Vector3f vec(p.x - pt[0], p.y - pt[1], p.z - pt[2]);
      float proj = vec.dot(axis);
      if (proj < min_proj) min_proj = proj;
      if (proj > max_proj) max_proj = proj;
  }

  height = max_proj - min_proj;
  radius = coefficients_cylinder->values[6];
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
    stereo->setDefaultProfilePreset(dai::node::StereoDepth::PresetMode::DEFAULT);
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
    int savedFileIndex = 0;
    while(true) {
        auto videoIn = videoQueue->get<dai::ImgFrame>();
        if(videoIn == nullptr) continue;

        cv::imshow("video", videoIn->getCvFrame());

        if(cv::waitKey(1) == 'q') {
            break;
        }
        else if (cv::waitKey(1) == 'r') 
        {
            auto image = videoQueue->get<dai::ImgFrame>();
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
    
            // Define the file name
            std::ostringstream oss;
            oss << std::setw(3) << std::setfill('0') << savedFileIndex;
            std::string result = oss.str();
            std::string fileName = "recordedPC_" + result;
            std::string fileNamePLY = "recordedPC_" + result + ".ply";
            std::string fileNamePCD = "recordedPC_" + result + ".pcd";
            std::string fileNameImage = "recordedPC_" + result + ".jpg";

            // Save the corresponding image
            cv::imwrite(fileNameImage, image->getCvFrame());

            // Save the point cloud
            if (pcl::io::savePLYFile(fileNamePLY, *cloud) != -1 && pcl::io::savePCDFile(fileNamePCD, *cloud) != -1)
            {
                std::cout << "Successfully saved the current PC to a file: " << fileName << std::endl;
                ++savedFileIndex;
            }
        }
        else if (cv::waitKey(1) == 't')
        {
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
    
            std::cout << "Cylinder radius: " << radius << "    Cylinder height: " << height << std::endl;
        }
    }
}