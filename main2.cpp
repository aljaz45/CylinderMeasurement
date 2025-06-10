#include <iostream>
#include <open3d/Open3D.h>
#include <depthai/depthai.hpp>

int main() {
    auto viewer = std::make_unique<pcl::visualization::PCLVisualizer>("Cloud Viewer");
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");

    dai::Device device(pipeline);

    auto q = device.getOutputQueue("out", 8, false);
    auto qDepth = device.getOutputQueue("depth", 8, false);

    while(true) {
        std::cout << "Waiting for data" << std::endl;
        auto depthImg = qDepth->get<dai::ImgFrame>();
        auto pclMsg = q->get<dai::PointCloudData>();
        
        if(!pclMsg) {
            std::cout << "No data" << std::endl;
            continue;
        }

        auto frame = depthImg->getCvFrame();
        frame.convertTo(frame, CV_8UC1, 255 / depth->initialConfig.getMaxDisparity());

        if(pclMsg->getPoints().empty()) {
            std::cout << "Empty point cloud" << std::endl;
            continue;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud = pclMsg->getPclData();
        viewer->updatePointCloud(cloud, "cloud");
        
        viewer->spinOnce(10);

        if(viewer->wasStopped()) {
            break;
        }
    }
}
