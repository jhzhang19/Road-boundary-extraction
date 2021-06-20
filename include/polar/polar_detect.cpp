#include<iostream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl-1.7/pcl/io/pcd_io.h>
#include <Eigen/Dense>


#include <string>
#include<math.h>
#include <iomanip>
#include <sstream>
#include <vector>

#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>

int main(){
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/zjh/文档/road_pcd/005404.pcd", *cloud);
    pcl::PointCloud<pcl::PointXYZ> left_point;
    pcl::PointCloud<pcl::PointXYZ> right_point;
    pcl::PointCloud<pcl::PointXYZ> right_curb;
    pcl::PointCloud<pcl::PointXYZ> left_curb;
    //将点云分块存储
    for(pcl::PointCloud<pcl::PointXYZ>::iterator it=cloud->begin();it!=cloud->end();++it){
        if(it->y<0){
            right_point.push_back(*it);
        }
        else if(it->y>0){
            left_point.push_back(*it);
        }
    }
    //cout<<right_point.size()<<"  "<<left_point.size()<<endl;
    /*right curb y<0*/
    float ang1 = atan2(8,3); //rad
    float ang2 = atan2(20,3);
    float delt = 0.2 * 3.14159 / 180;
    float ang_err = 0.08 * 3.14159 /180;
    for(float i=ang1;i<=ang2;i+=delt){
        pcl::PointXYZ point;
        float maxRadius = 0;
        for(pcl::PointCloud<pcl::PointXYZ>::iterator it=right_point.begin();it!=right_point.end();++it){
            float ang_i = atan2(it->x,(-1)*it->y);
            if(ang_i>=(i-ang_err)&&ang_i<=(i+ang_err)){
                float radius = sqrt(pow(it->x,2)+pow(it->y,2));
                if(radius>=maxRadius){
                    maxRadius = radius;
                    point.x = it->x;
                    point.y = it->y;
                    point.z = it->z;
                }


            }
        }
        right_curb.points.push_back(point);
    }
    cout<<right_curb.size()<<endl;

    /*left curb y>0*/
    float ang3 = atan2(8,5); //rad 车辆靠右行驶，左边宽度预留较宽距离
    float ang4 = atan2(20,5);
    //float delt = 0.2 * 3.14159 / 180;
    //float ang_err = 0.08 * 3.14159 /180;
    for(float i=ang3;i<=ang4;i+=delt){
        pcl::PointXYZ point;
        float maxRadius = 0;
        for(pcl::PointCloud<pcl::PointXYZ>::iterator it=left_point.begin();it!=left_point.end();++it){
            float ang_i = atan2(it->x,it->y);
            if(ang_i>=(i-ang_err)&&ang_i<=(i+ang_err)){
                float radius = sqrt(pow(it->x,2)+pow(it->y,2));
                if(radius>=maxRadius){
                    maxRadius = radius;
                    point.x = it->x;
                    point.y = it->y;
                    point.z = it->z;
                }


            }
        }
        left_curb.points.push_back(point);
    }
    
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients1(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers1(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;   
    seg.setOptimizeCoefficients(true);    
    seg.setModelType(pcl::SACMODEL_LINE);  
    seg.setMethodType(pcl::SAC_RANSAC);     
    seg.setDistanceThreshold(0.2);        
    seg.setInputCloud(left_curb.makeShared());               
    seg.segment(*inliers, *coefficients);
    float x0_l = coefficients->values[0];
    float y0_l = coefficients->values[1];
    float z0_l = coefficients->values[2];
    float a_l = coefficients->values[3];
    float b_l = coefficients->values[4];
    float c_l = coefficients->values[5];
    left_curb.clear();
    for(float x=5;x<=15;x+=0.5){
        pcl::PointXYZ point;
        point.x = x;
        point.y = b_l*(x-x0_l)/a_l + y0_l;
        point.z = c_l*(x-x0_l)/a_l + z0_l;
        //cout<<point.x<<point.y<<point.z<<endl;
        left_curb.points.push_back(point);
    }

    seg.setOptimizeCoefficients(true);    
    seg.setModelType(pcl::SACMODEL_LINE);  
    seg.setMethodType(pcl::SAC_RANSAC);     
    seg.setDistanceThreshold(0.2);        
    seg.setInputCloud(right_curb.makeShared());               
    seg.segment(*inliers1, *coefficients1);
    float x0_r = coefficients1->values[0];
    float y0_r = coefficients1->values[1];
    float z0_r = coefficients1->values[2];
    float a_r = coefficients1->values[3];
    float b_r = coefficients1->values[4];
    float c_r = coefficients1->values[5];
    right_curb.clear();
    for(float x=5;x<=15;x+=0.5){
        pcl::PointXYZ point;
        point.x = x;
        point.y = b_r*(x-x0_r)/a_r + y0_r;
        point.z = c_r*(x-x0_r)/a_r + z0_r;
        //cout<<point.x<<point.y<<point.z<<endl;
        right_curb.points.push_back(point);
    }
    //---------------------- Visualizer -------------------------------------------
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer);
viewer->setBackgroundColor(255,255,255);

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(cloud, 255, 255, 0);
viewer->addPointCloud(cloud, color_handler, "sample cloud");
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "sample cloud");

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handlerK(right_curb.makeShared(), 255, 0, 0);
viewer->addPointCloud(right_curb.makeShared(), color_handlerK, "point");
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "point");

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handlerK1(left_curb.makeShared(), 0, 255, 255);
viewer->addPointCloud(left_curb.makeShared(), color_handlerK1, "point1");
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "point1");



while (!viewer->wasStopped())
{
    viewer->spinOnce(100);
}
    return 0;

}