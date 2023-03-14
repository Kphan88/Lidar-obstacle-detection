/* \author Aaron Brown */
// Create simple 3d highway enviroment using PCL
// for exploring self-driving car sensors

#include "sensors/lidar.h"
#include "render/render.h"
#include "processPointClouds.h"
// using templates for processPointClouds so also include .cpp to help linker
#include "processPointClouds.cpp"

std::vector<Car> initHighway(bool renderScene, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    Car egoCar( Vect3(0,0,0), Vect3(4,2,2), Color(0,1,0), "egoCar");
    Car car1( Vect3(15,0,0), Vect3(4,2,2), Color(0,0,1), "car1");
    Car car2( Vect3(8,-4,0), Vect3(4,2,2), Color(0,0,1), "car2");	
    Car car3( Vect3(-12,4,0), Vect3(4,2,2), Color(0,0,1), "car3");
  
    std::vector<Car> cars;
    
    cars.push_back(egoCar);
    cars.push_back(car1);

    cars.push_back(car2);
    cars.push_back(car3);

    if(renderScene)
    {
        renderHighway(viewer);
        egoCar.render(viewer);
        car1.render(viewer);
        car2.render(viewer);
        car3.render(viewer);
    }

    return cars;
}


void simpleHighway(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display simple highway -----
    // ----------------------------------------------------
    
    // RENDER OPTIONS
    bool renderScene = false;
    bool renderCluster = true;

    std::vector<Car> cars = initHighway(renderScene, viewer);
    
    // TODO:: Create lidar sensor
    Lidar* lidar = new Lidar(cars,0);
    pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud = lidar->scan();

    //renderRays(viewer, lidar->position, pt_cloud);
    renderPointCloud(viewer, pt_cloud, "test", Color(255,0,0));

    // Instantiate on the stack
    ProcessPointClouds<pcl::PointXYZ> pointProcessor;

    // Instantiate on the heap
    //ProcessPointClouds<pcl::PointXYZ>* pointProcessor = new ProcessPointClouds<pcl:PointXYZ>();

    // render bounding box (NP)
    /*
    for(pcl::PointCloud<pcl::PointXY>::Ptr cluster : cloudClusters)
    {
        if(render_clusters)
        {
            std::cout<< "cluster size";
            pointProcessor->numPoints(cluster);
            renderPointCloud(viewer, cluster, "obstcloud"+ std::to_string(cluserId), colors[clusterId]);
            if (render_box)
            {
                Box box = pointProcessor -> BoundingBox(cluster);
                renderBox(viewer, box, clusterId);
            }
            ++clusterId;

        }
    }
    */
    // NP
    

   

    
    // TODO:: Create point processor
    // from video
    /*
    std::pair<pcl::PointCloud<pcl::PointXYZ>::Ptr, pcl::PointCloud<pcl::PointXYZ.::Ptr> segmentCloud=pointProcessor.SegmentPlane(inputCloud, 100, 0.2);
    renderPointCloud(viewer, segmentCloud.first, "obstCloud", Color(1,0,0)); 
    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0,1,0));

    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloudClusters = pointProcessor.Clustering
    //
    */

}




// Load real PCD 
void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    // ----------------------------------------------------
    // -----Open 3D viewer and display city block -----
    // ----------------------------------------------------

    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();
    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloud = pointProcessorI->loadPcd("/home/workspace/SFND_Lidar_Obstacle_Detection/src/sensors/data/pcd/data_1/0000000000.pcd");
    //renderPointCloud(viewer, inputCloud, "inputCloud");
    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud = pointProcessorI->FilterCloud(inputCloud, 0.1,
                                             Eigen::Vector4f(-20, -6, -3, 1),
                                             Eigen::Vector4f(25, 6.5, 3, 1));
    std::cout << "*** DEBUG: input cloud size *** " << inputCloud->points.size()<<std::endl;
        std::cout << "*** DEBUG: filteredCloud cloud size *** " << filteredCloud->points.size()<<std::endl;

                                             
    //renderPointCloud(viewer, filteredCloud, "filteredCloud"); 
    
    // segment obstacel and groud
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr,pcl::PointCloud<pcl::PointXYZI>::Ptr> segResult = pointProcessorI->SegmentPlane(filteredCloud, 100,0.1);
    // Render 2 segments: 
    //renderPointCloud(viewer, segResult.first, "groudCloud", Color(0,1,0));
    //renderPointCloud(viewer, segResult.second, "obstacleCloud", Color(1,0,0));

    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> obstacleClusters = pointProcessorI->Clustering(segResult.second, 0.5, 1, 20);

    std::cout << " Numbers of obstalce detected:  "<< obstacleClusters.size()<<std::endl;
    



}




//setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}


int main (int argc, char** argv)
{
    std::cout << "starting enviroment......" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    CameraAngle setAngle = XY;
    initCamera(setAngle, viewer);
    //simpleHighway(viewer);
    cityBlock(viewer);

    while (!viewer->wasStopped ())
    {
        viewer->spinOnce ();
    } 
}