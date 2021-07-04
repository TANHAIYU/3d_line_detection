/**
 * @file    LineDetectionApp.cpp
 *
 * @author  btran
 *
 * @date    2021-07-04
 *
 */

#include <iostream>

#include <pcl/io/pcd_io.h>

#include <3d_line_detection/3d_line_detection.hpp>

#include "Utility.hpp"

namespace
{
using PointCloudType = pcl::PointXYZ;
using PointCloud = pcl::PointCloud<PointCloudType>;
using PointCloudPtr = PointCloud::Ptr;
using HoughTransform = perception::HoughTransform<PointCloudType>;

auto viewer = util::initializeViewer();
}  // namespace

int main(int argc, char* argv[])
{
    if (argc != 2) {
        std::cerr << "Usage: [app] [path/to/pcd]" << std::endl;
        return EXIT_FAILURE;
    }

    const std::string pclFilePath = argv[1];
    PointCloudPtr cloud(new PointCloud);
    if (pcl::io::loadPCDFile(pclFilePath, *cloud) == -1) {
        std::cerr << "Failed to load pcl file" << std::endl;
        return EXIT_FAILURE;
    }

    viewer->addPointCloud<PointCloudType>(cloud, "original_cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "original_cloud");

    HoughTransform::Param param;
    HoughTransform transformer(param);

    pcl::ModelCoefficients lineModelCoefficients;
    HoughTransform::LineSegment3Ds lineSegments = transformer.run(cloud);

    std::cout << "number of detected line segments: " << lineSegments.size() << std::endl;

    const std::vector<std::array<double, 3>> colors = util::generateColorCharts(lineSegments.size());
    int count = 0;
    for (const auto& lineSegment : lineSegments) {
        std::string lineLabel = "line" + std::to_string(count);
        pcl::ModelCoefficients modifiedCoeffs = lineSegment.coeffs();
        const float scaleFactor = 2.5;
        modifiedCoeffs.values[0] -= scaleFactor * modifiedCoeffs.values[3];
        modifiedCoeffs.values[1] -= scaleFactor * modifiedCoeffs.values[4];
        modifiedCoeffs.values[2] -= scaleFactor * modifiedCoeffs.values[5];
        modifiedCoeffs.values[3] *= 2 * scaleFactor;
        modifiedCoeffs.values[4] *= 2 * scaleFactor;
        modifiedCoeffs.values[5] *= 2 * scaleFactor;

        viewer->addLine(modifiedCoeffs, lineLabel);
        viewer->setShapeRenderingProperties(pcl::visualization::PCL_VISUALIZER_LINE_WIDTH, 1, lineLabel);

        auto projectedCloud = lineSegment.projectPointsOnLine();
        std::string projectedLabel = "projected" + std::to_string(count);
        viewer->addPointCloud<PointCloudType>(projectedCloud, projectedLabel);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_COLOR, colors[count][0],
                                                 colors[count][1], colors[count][2], projectedLabel);
        viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 4, projectedLabel);
        count++;
    }

    while (!viewer->wasStopped()) {
        viewer->spinOnce();
    }

    return EXIT_SUCCESS;
}
