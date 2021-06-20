#ifndef MAP_LABEL_H
#define MAP_LABEL_H

#include <iostream>
#include <numeric>
#include <algorithm>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <termios.h>
#include <image_transport/image_transport.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "pcl_ros/transforms.h"
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <Eigen/Dense>
#include <opencv/cv.hpp>
#include <sys/types.h>  
#include <dirent.h> 
#include <string>
#include<math.h>
#include <iomanip>
#include <sstream>
#include <vector>
#include <fstream>

using namespace std;

/*
Map_label 类主要涉及：
    1.获取文件名读取数据。注意：这里是基于路面分割图像来读取各种数据的，所以要保证点云、车道线文件要与对应的图片文件名相同
    2.点云投影并提取出路面或者车道线对应位置的点云
    3.可视化点云
    4.提取结果投影到图片并保存，用于查看提取效果
*/

class Map_label
{
private:
    /* data */
    struct initial_parameters
    {
        /* data */
        cv::Mat camtocam_mat;
        cv::Mat cameraIn;
        cv::Mat RT;
        float maxX;
        float minX;
        float maxY;
        float minY;
        float maxZ;

    }i_params;
    
public:
     Map_label(){} 
    ~Map_label(){}

    
     void initParams()
    {
        /*
        获取配置参数：
        注意需要设置字符串流infile文件目录，最好使用绝对路径
        相机与雷达之间的标定参数：此处是利用的kitti数据测试，包括
        1.lidar->camera(RT)
        2. camera->camera(R-rect)
        3. camera内参(P-rect)
        具体使用根据实际在cfg/initial_params.txt中修改外参，每一行具体定义见文档
        剩余参数为截取点云ROI区域的参数
        */

        std::ifstream infile("/home/zjh/shared_dir/work/map_label/cfg/initial_params.txt");//注意设置
        double_t camtocam[16];
        double_t cameraIn[12];
        double_t RT[16];
        for (int i = 0; i < 16; i++){
            infile >> camtocam[i];
        }
        cv::Mat(4, 4, 6, &camtocam).copyTo(i_params.camtocam_mat);//cameratocamera params
        //std::cout<<i_params.camtocam_mat<<std::endl;

        for (int i = 0; i < 12; i++){
            infile >> cameraIn[i];
        }
        cv::Mat(3, 4, 6, &cameraIn).copyTo(i_params.cameraIn);//cameraIn params
        //std::cout<<i_params.cameraIn<<std::endl;

        for (int i = 0; i < 16; i++){
            infile >> RT[i];
        }
        cv::Mat(4, 4, 6, &RT).copyTo(i_params.RT);//lidar to camera params
        //std::cout<<i_params.RT<<std::endl;
        //the parm of crop lidar points
        infile >> i_params.maxX;
        infile >> i_params.minX;
        infile >> i_params.maxY;
        infile >> i_params.minY;
        infile >> i_params.maxZ;
       
    }

    
    pcl::PointCloud<pcl::PointXYZ>::Ptr ProjectAndExtract(cv::Mat raw_img, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        /*
        将点云投影到图像上，筛选出路面点：
        1.先截取ROI点云
        2.然后点云逐一投影，检查对应像素点的像素值是否大于阈值（此处阈值根据需要调整）
        3.筛选出点云保存
        */
        cv::Mat visImg = raw_img.clone();
        cv::Mat overlay = visImg.clone();
        //std::cout<<"get pc and image data:"<<cloud->size()<<std::endl;
        
        cv::Mat X(4,1,cv::DataType<double>::type);
        cv::Mat Y(3,1,cv::DataType<double>::type);
        pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_extr(new pcl::PointCloud<pcl::PointXYZ>);
        for(pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud->points.begin(); it != cloud->points.end(); it++){
            
            if(it->x > i_params.maxX || it->x < i_params.minX || it->y > i_params.maxY || it->y < i_params.minY || it->z > i_params.maxZ ) 
            {
                continue;
            }
            
            X.at<double>(0,0) = it->x;
            X.at<double>(1,0) = it->y;
            X.at<double>(2,0) = it->z;
            X.at<double>(3,0) = 1;

            Y = i_params.cameraIn * i_params.camtocam_mat * i_params.RT * X;  //tranform the point to the camera coordinate

            cv::Point pt;
            pt.x = Y.at<double>(0,0) / Y.at<double>(2,0);
            pt.y = Y.at<double>(1,0) / Y.at<double>(2,0);
           
            if(pt.x<=visImg.cols&&pt.x>=0&&pt.y<=visImg.rows&&pt.y>=0){
                int b = visImg.at<cv::Vec3b>(pt.y, pt.x)[0];
                int g = visImg.at<cv::Vec3b>(pt.y, pt.x)[1];
                int r = visImg.at<cv::Vec3b>(pt.y, pt.x)[2];
                if(b>75&&g>75&&r>75){
                    pcl::PointXYZ point;
                    point.x = it->x;
                    point.y = it->y;
                    point.z = it->z;      
                    pointcloud_extr->points.push_back(point);
                }

            }

        }
        return pointcloud_extr;
    }

    void ProjectAndVis(cv::Mat& raw_img, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool vis, string path)
    {
        /*
            此函数主要用于调试，将提取出来的点云重投影到原始RGB或者mask图都行，可以直接可视化或者输出保存
            bool vis == false ------ 输出保存   true -----可视化不保存
            注意输入图片的路径
        */
        cv::Mat visImg = raw_img.clone();
        cv::Mat overlay = visImg.clone();
        //std::cout<<"get pc and image data:"<<cloud->size()<<std::endl;
        
        cv::Mat X(4,1,cv::DataType<double>::type);
        cv::Mat Y(3,1,cv::DataType<double>::type);
        //pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud_extr(new pcl::PointCloud<pcl::PointXYZ>);
        for(pcl::PointCloud<pcl::PointXYZ>::const_iterator it = cloud->points.begin(); it != cloud->points.end(); it++){
            
            X.at<double>(0,0) = it->x;
            X.at<double>(1,0) = it->y;
            X.at<double>(2,0) = it->z;
            X.at<double>(3,0) = 1;

            Y = i_params.cameraIn * i_params.camtocam_mat * i_params.RT * X;  //tranform the point to the camera coordinate

            cv::Point pt;
            pt.x = Y.at<double>(0,0) / Y.at<double>(2,0);
            pt.y = Y.at<double>(1,0) / Y.at<double>(2,0);
            cv::circle(overlay, pt, 1, cv::Scalar(0,0,255), -1);
            
        }
        if(vis){
            string windowName = "curb on road";
            cv::namedWindow(windowName, 1); 
            cv::imshow(windowName, overlay);
            cv::waitKey(0);

        }
        else{
            cv::imwrite(path,overlay);
        }
        
        
    }
     
    void GetFileNames(string& path, vector<string>& filenames)
    {
        /*
        读取文件名：
        注意该函数读取文件名不一定是按照序号读取的
        返回该目录下文件名数组
        */
        DIR *pDir;
        struct dirent* ptr;
        if(!(pDir = opendir(path.c_str())))
            return;
        while((ptr = readdir(pDir))!=0) {
            if (strcmp(ptr->d_name, ".") != 0 && strcmp(ptr->d_name, "..") != 0)
                filenames.push_back(ptr->d_name);
        }
        closedir(pDir);
    }

    
    void Visualization(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_A)
    {
        /*
        点云可视化
        */
        // 初始化点云可视化界面
        boost::shared_ptr<pcl::visualization::PCLVisualizer>
        viewer_final (new pcl::visualization::PCLVisualizer ("3D Viewer"));
        viewer_final->setBackgroundColor (255, 255, 255);
        //对目标点云着色（红色）并可视化
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ>
        target_color (cloud_A, 255, 0, 0);
        viewer_final->addPointCloud<pcl::PointXYZ> (cloud_A, target_color, "target cloud");
        viewer_final->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                                                        5, "target cloud");
        // 启动可视化
        viewer_final->addCoordinateSystem (1.0);
        viewer_final->initCameraParameters ();

        //等待直到可视化窗口关闭。
        while (!viewer_final->wasStopped ())
        {
            viewer_final->spinOnce (10);
            boost::this_thread::sleep (boost::posix_time::microseconds (10));
        }

    }

    
    
};

#endif