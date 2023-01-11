//
// Created by rmt on 22-12-24.
//
#include <iostream>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/console/time.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/common/transforms.h>
#include <yaml-cpp/yaml.h>

using namespace std;
int main()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_2(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::console::TicToc tt;
    std::cerr<<"Reader...\n",tt.tic();

    pcl::PCDReader reader1;
    reader1.read("/home/kyland/ws_livox/src/FAST_LIO1/PCD/scans051.pcd",*cloud_1);
    pcl::PCDReader reader2;
    reader2.read("/home/kyland/ws_livox/src/FAST_LIO/PCD/scans551.pcd",*cloud_2);

    std::cerr<<"Done  "<<tt.toc()<<"  ms\n"<<std::endl;
    //转换
    YAML::Node config = YAML::LoadFile("/home/kyland/20221224/kyland-cc/551TRTK.yml");
    vector<float> ext = config["Lidar551TRTK"]["data"].as<vector<float>>();
    assert((int)ext.size() == 16);
    //cv::Mat Lidar2W = cv::Mat(4, 4, CV_64F);
    Eigen::Matrix4f Lidar2W = Eigen::Matrix4f::Identity();
    for (int row = 0; row < 4; row++)
    {
        for (int col = 0; col < 4; col++)
        {
            //Lidar2W.at<double>(row, col) = ext[col + row * 4];
            Lidar2W(row, col) = ext[col + row * 4];
        }
    }
    cout << "外参矩阵："<< setprecision(15) << Lidar2W << endl;
    cout << "原数据："<< config["Lidar551TRTK"]["data"] << endl;

    /*Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();//单位矩阵
    transform_1 (0,0) = 8.7174797058105469e-001;
    transform_1 (0,1) = 1.6136988997459412e-003;
    transform_1 (0,2) = -4.8995223641395569e-001;
    transform_1 (0,3) = -2.4747371673583984e-001;
    transform_1 (1,0) = -2.0353859290480614e-003;
    transform_1 (1,1) = 9.9999779462814331e-001;
    transform_1 (1,2) = -3.2803788781166077e-004;
    transform_1 (1,3) = 2.3969173431396484e-002;
    transform_1 (2,0) = 4.8995062708854675e-001;
    transform_1 (2,1) = 1.2832358479499817e-003;
    transform_1 (2,2) = 8.7174940109252930e-001;
    transform_1 (2,3) = 2.2653722763061523e-001;
    transform_1 (3,0) = 0;
    transform_1 (3,1) = 0;
    transform_1 (3,2) = 0;
    transform_1 (3,3) = 1;*/

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudOut_1(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_1, *cloudOut_1, Lidar2W);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_x(new pcl::PointCloud<pcl::PointXYZ>);
    *cloud_x = *cloudOut_1 + *cloud_2;



//    std::cerr<<"The point cloud_1 has:  "<<cloud_1.points.size()<<"  points data."<<std::endl;
//    std::cerr<<"The point cloud_2 has:  "<<cloud_2.points.size()<<"  points data."<<std::endl;
//    std::cerr<<"The point cloud_3 has:  "<<cloud_3.points.size()<<"  points data."<<std::endl;

    pcl::PCDWriter writer;
    writer.write("/home/kyland/230110pinjie.pcd",*cloud_x);
/*
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_7(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PCDReader reader_7;
    reader_7.read("/home/kyland/pinjie.pcd",*cloud_7);
    std::cerr<<cloud_7,tt.tic();
    std::cerr<<"SorFilter...\n",tt.tic();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::VoxelGrid<pcl::PointXYZ> sor;
    sor.setInputCloud(cloud_7);
    sor.setLeafSize(0.01f,0.01f,0.01f);
    sor.filter(*cloud_filtered);
    std::cerr<<"Done  "<<tt.toc()<<" ms.\n"<<std::endl;

    std::cerr<<"视图...\n",tt.tic();
//    pcl::visualization::CloudViewer viewer("3D Viewer");
//     std::cerr<<"1111  "<<std::endl;
//    viewer.showCloud(cloud_filtered);
//     std::cerr<<"2222  "<<std::endl;
    pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor (0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ> (cloud_filtered, "sample cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "sample cloud");
    viewer->addCoordinateSystem (1.0);
    viewer->initCameraParameters ();
    while(!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
    */
    std::cerr<<"Done  "<<tt.toc()<<"  ms.\n"<<std::endl;

    return 0;
}