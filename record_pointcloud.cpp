#include <csignal>
#include <depthai/remote_connection/RemoteConnection.hpp>
#include <iostream>

#include <open3d/Open3D.h>
#include "depthai/depthai.hpp"

// Upgraded visualizer_rgbd.cpp example, with added functionality of recording
// point cloud data and storing it to a file

// Signal handling for clean shutdown
static bool isRunning = true;
void signalHandler(int signum) {
    isRunning = false;
}

int main() {
    using namespace std;
    // Default port values
    int webSocketPort = 8765;
    int httpPort = 8080;

    // Register signal handler
    std::signal(SIGINT, signalHandler);

    // Create RemoteConnection
    dai::RemoteConnection remoteConnector(dai::RemoteConnection::DEFAULT_ADDRESS, webSocketPort, true, httpPort);
    // Create pipeline
    dai::Pipeline pipeline;
    auto left = pipeline.create<dai::node::Camera>();
    auto right = pipeline.create<dai::node::Camera>();
    auto stereo = pipeline.create<dai::node::StereoDepth>();
    auto rgbd = pipeline.create<dai::node::RGBD>();
    auto color = pipeline.create<dai::node::Camera>();
    auto pointcloud = pipeline.create<dai::node::PointCloud>();
    std::shared_ptr<dai::node::ImageAlign> align;
    color->build();

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
    if(platform == dai::Platform::RVC4) {
        auto* out = color->requestOutput(std::pair<int, int>(640, 400), dai::ImgFrame::Type::RGB888i);
        out->link(rgbd->inColor);
        align = pipeline.create<dai::node::ImageAlign>();
        stereo->depth.link(align->input);
        out->link(align->inputAlignTo);
        align->outputAligned.link(rgbd->inDepth);
    } else {
        auto* out = color->requestOutput(std::pair<int, int>(640, 400), dai::ImgFrame::Type::RGB888i, dai::ImgResizeMode::CROP, 30, true);
        out->link(rgbd->inColor);
        out->link(stereo->inputAlignTo);
        stereo->depth.link(rgbd->inDepth);
    }

    // Point cloud
    pointcloud->initialConfig->setSparse(true);
    stereo->depth.link(pointcloud->inputDepth);
    auto queue = pointcloud->outputPointCloud.createOutputQueue();
    auto rgbdPclQueue = rgbd->pcl.createOutputQueue();

    remoteConnector.addTopic("pcl", rgbd->pcl);
    pipeline.start();
    remoteConnector.registerPipeline(pipeline);
    auto device = pipeline.getDefaultDevice();
    device->setIrLaserDotProjectorIntensity(0.7);

    // Main loop
    int savedFileIndex = 0;
    while(isRunning && pipeline.isRunning()) 
    {
        // Check for a key press
        int key = remoteConnector.waitKey(1);
        if(key == 'q') 
        {
            std::cout << "Got 'q' key from the remote connection!" << std::endl;
            break;
        }
        else if (key == 'r') 
        {
            auto pcl = queue->get<dai::PointCloudData>();
            //auto pcl = rgbdPclQueue->get<dai::PointCloudData>();
            if (pcl == nullptr)
                continue;

            // Convert to Eigen::Vector3d   TO DO: Probably a better way to do this ...
            std::vector<Eigen::Vector3d> convertedPoints;
            convertedPoints.reserve(pcl->getPoints().size());
            for (const auto& point : pcl->getPoints()) 
            {
                convertedPoints.emplace_back(static_cast<double>(point.x),
                                             static_cast<double>(point.y),
                                             static_cast<double>(point.z));
            }
    
            // Create Open3D point cloud and assign converted points to it
            open3d::geometry::PointCloud pcl3D;
            pcl3D.points_ = convertedPoints;

            // Define the file name
            std::ostringstream oss;
            oss << std::setw(3) << std::setfill('0') << savedFileIndex;
            std::string result = oss.str();
            std::string fileName = "recordedPC_" + result + ".ply";

            // Save the point cloud
            if (open3d::io::WritePointCloud(fileName, pcl3D))
            {
                std::cout << "Successfully saved the current PC to a file: " << fileName << std::endl;
                ++savedFileIndex;
            }
        }
    }

    std::cout << "Pipeline stopped." << std::endl;
    return 0;
}
