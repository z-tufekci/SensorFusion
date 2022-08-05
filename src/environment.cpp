/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr &viewer)
{

    Car egoCar(Vect3(0, 0, 0), Vect3(4, 2, 2), Color(0, 1, 0), "egoCar");
    Car car1(Vect3(15, 0, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car1");
    Car car2(Vect3(8, -4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car2");
    Car car3(Vect3(-12, 4, 0), Vect3(4, 2, 2), Color(0, 0, 1), "car3");

    std::vector<Car> cars;
    cars.push_back(egoCar);
    cars.push_back(car1);
    cars.push_back(car2);
    cars.push_back(car3);

    if (renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}

void simpleHighway(pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------

    // RENDER OPTIONS
    bool renderScene = false;
    std::vector<Car> cars = initHighway(renderScene, viewer);

    // TODO:: Create lidar sensor
    Lidar *lidar = new Lidar(cars, 0);

    // TODO:: Create point processor
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointer = lidar->scan();
    //renderRays(viewer , lidar -> position , pointer);

    //ProcessPointClouds<pcl::PointXYZ> ppcl;
    ProcessPointClouds<pcl::PointXYZ> pointProcessor; // =  new ProcessPointClouds<pcl::PointXYZ>();
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segCloud = pointProcessor.SegmentPlane(pointer, 100, 0.2);
    //renderPointCloud(viewer, segCloud.first, "obs" ,Color(1,0,0));
    //renderPointCloud(viewer, segCloud.second, "plane" ,Color(1,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering(segCloud.first, 2.0, 3, 50);
    /*
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        std::cout << "cluster size";
        pointProcessor.numPoints(cluster);
        renderPointCloud(viewer, cluster, "obsCloud" + std::to_string(clusterId), colors[clusterId]);

        Box box = pointProcessor.BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
    */
}

void cityBlock(pcl::visualization::PCLVisualizer::Ptr &viewer, ProcessPointClouds<pcl::PointXYZ> *pointprocessor, const pcl::PointCloud<pcl::PointXYZ>::Ptr &inputCloud)
{
    /*pointprocessorI = new ProcessPointClouds<pcl::PointXYZI>();
      pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointprocessorI -> loadPcd("../src/sensors/data/pcd/data_1/0000000000.pcd");
      renderPointCloud(viewer, inputCloud, "inputCloud");*/

    pcl::PointCloud<pcl::PointXYZ>::Ptr filterCloud = pointprocessor->FilterCloud(inputCloud, 0.15f, Eigen::Vector4f(-20, -6, -5, 1), Eigen::Vector4f(30, 7, 5, 1));

    //SEGMENTATION TO plane and obstacle region with RANSAC
    // I decrease maxIteration from 100 to 50. It still works fine. It renders faster than before.
    // I thought Clustering and Ransac could be improvable 
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ>::Ptr> segCloud = pointprocessor->SegmentPlane(filterCloud, 50, 0.2);

    //renderPointCloud(viewer, segCloud.first, "obs", Color(1, 0, 0));
    renderPointCloud(viewer, segCloud.second, "plane", Color(1, 1, 0));

    
    // ECLUDIAN CLUSTERING
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointprocessor->Clustering(segCloud.first, 0.42f, 18, 1000);
    
    int clusterId = 0;
    std::vector<Color> colors = {Color(1, 0, 0), Color(0, 1, 0), Color(0, 0, 1), Color(0, 1, 1), Color(1, 0, 1)};

    for (pcl::PointCloud<pcl::PointXYZ>::Ptr cluster : cloudClusters)
    {
        //std::cout << "cluster size";
        //pointprocessor->numPoints(cluster);
        renderPointCloud(viewer, cluster, "obsCloud" + std::to_string(clusterId), colors[clusterId % (colors.size())]);
        Box box = pointprocessor->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);
        ++clusterId;
    }
    
}

//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr &viewer)
{
    viewer->setBackgroundColor(0, 0, 0);

    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;

    switch (setAngle)
    {
    case XY:
        viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0);
        break;
    case TopDown:
        viewer->setCameraPosition(0, 0, distance, 1, 0, 1);
        break;
    case Side:
        viewer->setCameraPosition(0, -distance, 0, 0, 0, 1);
        break;
    case FPS:
        viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char **argv)
{
    std::cout << "starting enviroment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    ProcessPointClouds<pcl::PointXYZ> *pointprocessor = new ProcessPointClouds<pcl::PointXYZ>();
    std::vector<boost::filesystem::path> stream = pointprocessor->streamPcd("../src/sensors/data/pcd/data_1");
    auto streamIterator = stream.begin();

    pcl::PointCloud<pcl::PointXYZ>::Ptr inputCloud;

    while (!viewer->wasStopped())
    {
        //Clear viewer
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        inputCloud = pointprocessor->loadPcd((*streamIterator).string());
        cityBlock(viewer, pointprocessor, inputCloud);

        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();
        viewer->spinOnce();
    }
}