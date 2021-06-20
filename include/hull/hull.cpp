// chainHull_2D(): Andrew's monotone chain 2D convex hull algorithm
//     Input:  P[] = an array of 2D points
//                  presorted by increasing x and y-coordinates
//             n =  the number of points in P[]
//     Output: H[] = an array of the convex hull vertices (max is n)
//     Return: the number of points in H[]
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

#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <unordered_map>


bool comp_up(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
{
    return a.y < b.y;
}

bool comp_down(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
{
    return a.y > b.y;
}
using namespace std;
 int hiPoints_WhereAreYouFrom(pcl::PointXYZ p)
    {
        /*velodyne 64e*/
        // find which to which layer the point belongs
        // refer to this code to understand it：https://github.com/luhongquan66/loam_velodyne/blob/master/src/lib/MultiScanRegistration.cpp
        double angle;
        int scanID;
        
        //{1， 1}, {3,3}, {5,5}, {7,7}, {9,9}, {11,11}, {13,13}, {15,15}, {-1, 14}, {-3, 12}, {-5,10}, {-7, 8}
        //{-9, 6}, {-11, 4}, {-13, 2}, {-15, 0}
        
        angle = atan(p.z / sqrt(p.x * p.x + p.y * p.y));
        angle = angle * 180 / 3.1415926;
        // if(angle>0){
        //     double delt = angle - floor(angle);
        //     if(delt>0.5){
        //         scanID = ceil(angle);
        //     }
        //     else{
        //         scanID = floor(angle);
        //     }
        // }
        // else{
        //     double delt = abs(angle - ceil(angle)); //abs(-15.6-(-15))
        //     if(delt>0.5){
        //         scanID = floor(angle) + 15;
        //     }
        //     else{
        //         scanID = ceil(angle) + 15;
        //     }
        // }
        // if(scanID<0)scanID = 0;
        //if(scanID>15)scanID = 15;
         /*速腾RS-16*/
        std::unordered_map<float,int> idMap {{1,16}, {3,15}, {5,14}, {7,13}, {9,12}, {11,11}, {13,10}, {15,9}, {-1,8}, {-3,7}, {-5,6}, {-7,5}, {-9,4}, {-11,3}, {-13,2}, {-15,1}};
        
        if(angle>0){
            double delt = angle - floor(angle);
            if(delt>0.5){
                scanID = ceil(angle) > 15 ? 15 : idMap[ceil(angle)];
            }
            else{
                scanID = floor(angle) < 1 ? 1 : idMap[floor(angle)];
            }
        }
        else{//angle < 0
            double delt = abs(angle - ceil(angle)); //abs(-15.6-(-15))
            if(delt>0.5){
                scanID = floor(angle) < -15 ? -15 : idMap[floor(angle)];
            }
            else{
                scanID = ceil(angle) > -1 ? -1 : idMap[ceil(angle)];
            }
        }
        return scanID;
    }
int main(){
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ> test_pc;
    pcl::io::loadPCDFile<pcl::PointXYZ>("/home/zjh/桌面/data_process/data/raw_data/0.pcd", *cloud);
    for(pcl::PointCloud<pcl::PointXYZ>::iterator it=cloud->begin();it!=cloud->end();++it){
        pcl::PointXYZ point;
        point.x = it->x;
        point.y = it->y;
        point.z = it->z;
        int id = hiPoints_WhereAreYouFrom(point);
        cout<<id<<endl;
        if(id==4||id==3||id==7){
            test_pc.points.push_back(point);
        }
    }

/*
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::io::loadPCDFile<pcl::PointXYZ>("/home/zjh/桌面/data_process/data/raw_data/0.pcd", *cloud);

pcl::ConvexHull<pcl::PointXYZ> hull;                  
hull.setInputCloud(cloud);                   
hull.setDimension(2);

std::vector<pcl::Vertices> polygons;                 
pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ> left_point;
pcl::PointCloud<pcl::PointXYZ> right_point;
// pcl::PointCloud<pcl::PointXYZ>::Ptr left_curb(new pcl::PointCloud<pcl::PointXYZ>);
// pcl::PointCloud<pcl::PointXYZ>::Ptr right_curb(new pcl::PointCloud<pcl::PointXYZ>);
pcl::PointCloud<pcl::PointXYZ> left_curb;
pcl::PointCloud<pcl::PointXYZ> right_curb;
vector<pcl::PointXYZ> vc;
hull.reconstruct(*surface_hull, polygons); 
for(pcl::PointCloud<pcl::PointXYZ>::iterator it=surface_hull->begin();it!=surface_hull->end();it++){    
    pcl::PointXYZ point;
    point.x = it->x;
    point.y = it->y;
    point.z = it->z;
    if(it->y>0){
        left_point.points.push_back(point);
    }
    else if(it->y<0){
        right_point.points.push_back(point);
    }
    
} 
sort(left_point.begin(),left_point.end(),comp_down);
sort(right_point.begin(),right_point.end(),comp_up);
// for(pcl::PointCloud<pcl::PointXYZ>::iterator it=left_point->begin();it!=left_point->end();it++){
//      cout<<it->x<<" "<<it->y<<" "<<it->z<<endl;}
int len_l = left_point.size();
int len_r = right_point.size();
for(int i=0;i<len_l-1;i++){
    float abs_ = abs(left_point[i].y - left_point[i+1].y);
    if(abs_<0.6){
        int len = left_curb.size();
        if(len>0){
            float tem = left_curb[len-1].y;
            int dy = abs(tem-left_point[i].y);
            if(dy>0.5){
                continue;
            }
        }
        left_curb.points.push_back(left_point[i]);
        //cout<<left_point[i].y<<endl;
    }
    
}
cout<<"left_curb"<<left_curb.size()<<endl;
for(int i=0;i<len_r-1;i++){
    float abs_ = abs(right_point[i].y - right_point[i+1].y);
    if(abs_<0.5){
        int len = right_curb.size();
        if(len>0){
            float tem = right_curb[len-1].y;
            int dy = abs(tem-right_point[i].y);
            if(dy>0.5){
                continue;
            }
        }
        right_curb.points.push_back(right_point[i]);
        
    }
    
}
cout<<"right_curb"<<right_curb.size()<<endl;

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

// for(pcl::PointCloud<pcl::PointXYZ>::iterator it=surface_hull->begin();it!=surface_hull->end();it++){
//     cout<<it->x<<" "<<it->y<<" "<<it->z<<endl;
// }
std::cout << "a：" << coefficients->values[0] << endl;
    std::cout << "b：" << coefficients->values[1] << endl;
    std::cout << "c：" << coefficients->values[2] << endl;
    std::cout << "d：" << coefficients->values[3] << endl;
    std::cout << "e：" << coefficients->values[4] << endl;
    std::cout << "f：" << coefficients->values[5] << endl;


cout << surface_hull->size() << endl;
*/
//---------------------- Visualizer -------------------------------------------
boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer (new pcl::visualization::PCLVisualizer);
viewer->setBackgroundColor(255,255,255);

pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handler(test_pc.makeShared(), 255, 255, 0);
viewer->addPointCloud(test_pc.makeShared(), color_handler, "sample cloud");
viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "sample cloud");

// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handlerK(left_curb.makeShared(), 10, 0, 50);
// viewer->addPointCloud(left_curb.makeShared(), color_handlerK, "point");
// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "point");

// pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_handlerK1(right_curb.makeShared(), 0, 255, 255);
// viewer->addPointCloud(right_curb.makeShared(), color_handlerK1, "point1");
// viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 6, "point1");

// viewer->addPolygon<pcl::PointXYZ>(surface_hull, 0, 0, 255, "polyline");

while (!viewer->wasStopped())
{
    viewer->spinOnce(100);
}


return 0;
}

  

 
