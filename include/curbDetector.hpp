#ifndef CURBDETECTOR_H
#define CURBDETECTOR_H

#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <math.h>
#include <vector>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/crop_hull.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <unordered_map>
using namespace std;

bool comp_up(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
{
    return a.y < b.y;
}

bool comp_down(const pcl::PointXYZ &a, const pcl::PointXYZ &b)
{
    return a.y > b.y;
}

// This class is used to detect curb. 
// Input a point cloud and return a point cloud consisting of detected curb points.
class curbDetector
{
public: 
    struct initial_parameters
    {
        /* data */
        int ringNums;
        vector<int> ring_idx_;
        int pointNumthre;
        float filter_r;
        int k;
        int mean_k;
        float stdthre;
        string road_mask_img;
        string pc_pcd;
        string lane_mask_img;
        string original_rgb_img;
        string res_vis_path;
        string road_pcd;
        string lane_pcd;
        
    }curbDetect_params;

    std::vector<pcl::PointCloud<pcl::PointXYZ> > pc_in;
    pcl::PointCloud<pcl::PointXYZ> cloud_in;
    pcl::PointCloud<pcl::PointXYZ> cloud_out;
    std::vector<pcl::PointCloud<pcl::PointXYZ> > cloud_cleaned;
    curbDetector(){}
    ~curbDetector(){}

    void initParam(){
        /*
            获取边界提取的相关设置参数
            注意设置配置文件目录，最好使用绝对路径
            参数主要包含：
            1. 挑选的线束数量，比如设置15条，就要设置相应的15个序号
            2.线束ID号
            3.每条线束最少点数阈值
            4.半径滤波，主要用于路面点云提取之后筛除离群点
            5.统计滤波参数
            6.输入文件路径及输出文件路径
        */
        std::ifstream infile("/home/zjh/shared_dir/work/map_label/cfg/curb_initial_params.txt");
        infile >> curbDetect_params.ringNums;   //number of the ring
        //cout<<"curbDetect_params.ringNums:"<<curbDetect_params.ringNums<<endl;
        curbDetect_params.ring_idx_.resize(curbDetect_params.ringNums);
        for(int i=0;i<curbDetect_params.ringNums;i++){
            infile >> curbDetect_params.ring_idx_[i];
            //cout<<"curbDetect_params.ring_idx_[i]"<<curbDetect_params.ring_idx_[i]<<endl;
        }
        infile >> curbDetect_params.pointNumthre;   // the threshould of the point number in every ring
        //cout<<"curbDetect_params.pointthre"<< curbDetect_params.pointNumthre<<endl;
        infile >> curbDetect_params.filter_r;   // the radius of the radius_filter
        infile >> curbDetect_params.k;          //the point number of k_nearest 
        infile >> curbDetect_params.mean_k;     //the mean point number in static filter
        infile >> curbDetect_params.stdthre;    //the stdevthreshould in static filter
        infile >> curbDetect_params.road_mask_img;
        infile >> curbDetect_params.pc_pcd;
        infile >> curbDetect_params.lane_mask_img;
        infile >> curbDetect_params.original_rgb_img;
        infile >> curbDetect_params.res_vis_path;
        infile >> curbDetect_params.road_pcd;
        infile >> curbDetect_params.lane_pcd;
    }

    pcl::PointCloud<pcl::PointXYZ> detector(const std::vector<pcl::PointCloud<pcl::PointXYZ> >& input,bool road_detect)
    {
        /*
            该函数是“分线束”提取路面边界点云或者车道线点云，主要使用bool road_detect作为标志
            true----提取道路边界
            false----提取车道线点云
            1.道路边界提取每条线束上的两端最外侧点
            2.车道线提取则提取每条线束上的中心点
            3.对提取之后的点利用ransac拟合直线后输出点
        */
        pcl::PointCloud<pcl::PointXYZ> curb_left;
        pcl::PointCloud<pcl::PointXYZ> curb_right;
        pc_in = input;
        //pcl::PointCloud<pcl::PointXYZ> look_test;
        int ring_num_ = curbDetect_params.ringNums;
        
        // Process and detect each layer
        if(road_detect){        
            //提取道路边界点
            for (int i = 0; i < ring_num_; i++)
            {
                pcl::PointCloud<pcl::PointXYZ> pointsInTheRing = pc_in[i]; // Save the points on this line.

                pcl::PointCloud<pcl::PointXYZ> left_pointcloud; 
                pcl::PointCloud<pcl::PointXYZ> right_pointcloud; 

                int numOfPointsInTheRing = pointsInTheRing.size();
                
                //cout<< numOfPointsInTheRing<<endl;
                if(numOfPointsInTheRing < curbDetect_params.pointNumthre ){
                    continue;
                }
                // Sort. (In ascending order of absolute value)
                sort(pointsInTheRing.begin(), pointsInTheRing.end(), comp_down);//y>0 big->small
                //cout<<pointsInTheRing.size()<<endl;
                curb_left.push_back(pointsInTheRing[0]);
                curb_right.push_back(pointsInTheRing[numOfPointsInTheRing-1]);

            }

        }
        else{
            //提取车道线点
            for (int i = 0; i < ring_num_; i++)
            {
                pcl::PointCloud<pcl::PointXYZ> pointsInTheRing = pc_in[i]; // Save the points on this line.
                pcl::PointCloud<pcl::PointXYZ> left_pointcloud;
                pcl::PointCloud<pcl::PointXYZ> right_pointcloud;
                int numOfPointsInTheRing = pointsInTheRing.size();
                if(numOfPointsInTheRing < 2 ){
                    continue;
                }
                for(pcl::PointCloud<pcl::PointXYZ>::iterator it=pointsInTheRing.begin();it!=pointsInTheRing.end();++it){
                    if(it->y>0)left_pointcloud.push_back(*it);  //对于双车道仅保存左侧车道线
                    // else{
                    //     right_pointcloud.push_back(*it);
                    // }
                }

                if(left_pointcloud.size()>0){
                    sort(left_pointcloud.begin(), left_pointcloud.end(), comp_up);//y>0 small->large
                    int num = (left_pointcloud.size()-1) / 2;
                    curb_left.push_back(left_pointcloud[num]);
                }
                if(right_pointcloud.size()>0){
                    sort(right_pointcloud.begin(), right_pointcloud.end(), comp_down);//y<0 large->small
                    int num = (left_pointcloud.size()-1) / 2;
                    curb_right.push_back(right_pointcloud[num]);

                }
                
            }

        }
        
    curbFit(curb_left,curb_right);
    return curb_left + curb_right;

    }

pcl::PointCloud<pcl::PointXYZ> PointFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr input, int k, float radius){

    /*
        该函数主要用于点云离群点滤除
        1.半径滤波
        2.统计滤波（未使用）
    */
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> outrem;
        // 创建滤波器
        outrem.setInputCloud(input);
        outrem.setRadiusSearch(radius);//搜索半径
        outrem.setMinNeighborsInRadius (k);//最少相邻点数
        // 应用滤波器
        outrem.filter (*cloud_filtered);
        
        // pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
        // sor.setInputCloud (input);
        // sor.setMeanK (curbDetect_params.mean_k);//设置在进行统计时考虑的查询点临近点数50
        // sor.setStddevMulThresh (curbDetect_params.stdthre);//标准差倍数，意味着如果一个点的距离超出平均距离一个标准差以上，则被标记为离群点
        // sor.filter (*cloud_filtered);

        return *cloud_filtered;
    }



    int hiPoints_WhereAreYouFrom(pcl::PointXYZ p)
    {
       //该函数用于确定每个点所在的线束ID
        //1.Velodyne 64e
        //2.rs-16
        //3.vlp-16
        //各类型雷达参数均从用户手册获取，对应用户手册见附件
        //目前默认为velodyne64，根据需求选用

        double angle;
        int scanID;
////---------------------------------------------------------------------------------------------------------------///
         /*velodyne 64e*/
        angle = atan(p.z / sqrt(p.x * p.x + p.y * p.y));
        scanID = (int)(angle * 134.18714161056457 + 58.81598513011153);
////---------------------------------------------------------------------------------------------------------------///
        /*速腾RS-16*/
        /*
        std::unordered_map<float,int> idMap {{1,16}, {3,15}, {5,14}, {7,13}, {9,12}, {11,11}, {13,10}, {15,9}, {-1,8}, {-3,7}, {-5,6}, {-7,5}, {-9,4}, {-11,3}, {-13,2}, {-15,1}};
        angle = atan(p.z / sqrt(p.x * p.x + p.y * p.y));
        angle = angle * 180 / 3.1415926;
        if(angle>0){
            double delt = angle - floor(angle);
            if(delt>0.5){
                scanID = ceil(angle) > 15 ? 15 : idMap[ceil(angle)];
            }
            else{
                scanID = floor(angle) < 1 ? 1 : idMap[floor(angle)];
            }
        }
        else{       //angle < 0
            double delt = abs(angle - ceil(angle)); //abs(-15.6-(-15))
            if(delt>0.5){
                scanID = floor(angle) < -15 ? -15 : idMap[floor(angle)];
            }
            else{
                scanID = ceil(angle) > -1 ? -1 : idMap[ceil(angle)];
            }
        }
        */

    ////---------------------------------------------------------------------------------------------------------------///
        /*vel0dyne vlp-16*/
        /*
        角度-线束序号对应
        {1， 1}, {3,3}, {5,5}, {7,7}, {9,9}, {11,11}, {13,13}, {15,15}, {-1, 14}, {-3, 12}, {-5,10}, {-7, 8} {-9, 6}, {-11, 4}, {-13, 2}, {-15, 0}
        */

        /*
        angle = atan(p.z / sqrt(p.x * p.x + p.y * p.y));
        angle = angle * 180 / 3.1415926;
        if(angle>0){
            double delt = angle - floor(angle);
            if(delt>0.5){
                scanID = ceil(angle);
            }
            else{
                scanID = floor(angle);
            }
        }
        else{
            double delt = abs(angle - ceil(angle)); //abs(-15.6-(-15))
            if(delt>0.5){
                scanID = floor(angle) + 15;
            }
            else{
                scanID = ceil(angle) + 15;
            }
        }
        if(scanID<0)scanID = 0;
        if(scanID>15)scanID = 15;
        */
  ////---------------------------------------------------------------------------------------------------------------///

        return scanID;
    }


    std::vector<pcl::PointCloud<pcl::PointXYZ> > cleanPoints(pcl::PointCloud<pcl::PointXYZ> pc)
    {
        //  该函数是将未挑选的线束清除，保留挑选的线束点云
        // 1. 获取线束序号，序号在cfg/curb_initial_param.txt文件中定义
        // 2. 根据点坐标调用hiPoints_WhereAreYouFrom获取laserid
        // 3. 保存需要的线束的点云
        const int cloudSize = pc.size();
        pcl::PointXYZ point;
        int scanID_;
        int ring_num_ = curbDetect_params.ringNums;
        
        std::vector<pcl::PointCloud<pcl::PointXYZ> > laserCloudScans(ring_num_);

        for (int i = 0; i < cloudSize; i++)
        {
            point.x = pc[i].x;
            point.y = pc[i].y;
            point.z = pc[i].z;

            if (!pcl_isfinite(point.x) || 
            !pcl_isfinite(point.y) || 
            !pcl_isfinite(point.z)) //过滤无效点
            {continue;}

            scanID_ = hiPoints_WhereAreYouFrom(point);
            
            for (int ring_num = 0;ring_num < ring_num_; ring_num++)
            {
                if (scanID_ == curbDetect_params.ring_idx_[ring_num])
                {
                    laserCloudScans[ring_num].push_back(point);
                    //cout<<point.x<<" "<<point.y<<" "<<point.z<<endl;
                }
            }
        }

        return laserCloudScans;
    }


pcl::PointCloud<pcl::PointXYZ> polar_detect(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool state){
    /*
            该函数为利用极坐标方式筛选边界点，该方法不需要分线束，对于同一个坐标系下的雷达数据可以尝试使用
            ang1/ ang2、ang3、 ang4 、delt、ang_err为重要参数，需要根据需求调整
            state---true 分割道路边沿，false 提取车道线点。此处内置的车道线提取方法是直接对车道线点进行拟合输出
    */
    pcl::PointCloud<pcl::PointXYZ> right_curb;
    pcl::PointCloud<pcl::PointXYZ> left_curb;
    if(state){
        
        pcl::PointCloud<pcl::PointXYZ> left_point;
        pcl::PointCloud<pcl::PointXYZ> right_point;
        
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
        float ang1 = atan2(8,2.5);  //右侧极轴开始扫描角度  8：开始扫描点的x坐标  2.5：开始扫描点的y坐标（原本为负，为处理方便转为正）
        float ang2 = atan2(30,2.5); //右侧极轴结束扫描角度
        float delt = 0.4 * 3.14159 / 180;   //扫描角度分辨率0.4度
        float ang_err = 0.09 * 3.14159 / 180;    //扫描角度公差范围  0.09度
        for(float i=ang1;i<=ang2;i+=delt){
            pcl::PointXYZ point;
            float maxRadius = 0;
            for(pcl::PointCloud<pcl::PointXYZ>::iterator it=right_point.begin();it!=right_point.end();++it){
                float ang_i = atan2(it->x,(-1)*it->y);
                if(ang_i>=(i-ang_err)&&ang_i<=(i+ang_err)){
                    float radius = sqrt(pow(it->x,2)+pow(it->y,2));
                    if(radius>=maxRadius){
                        maxRadius = radius;     //筛选极轴最长的点
                        point.x = it->x;
                        point.y = it->y;
                        point.z = it->z;
                    }


                }
            }
            if(maxRadius<5)continue;    //筛除近处的因重复扫描出现的点
            right_curb.points.push_back(point);
        }
        right_curb = PointFilter(right_curb.makeShared(),2,0.5);


        /*left curb y>0*/
        float ang3 = atan2(8,8); //rad 车辆靠右行驶，左边宽度预留较宽距离
        float ang4 = atan2(40,8);
        for(float i=ang3;i<=ang4;i+=delt){
            pcl::PointXYZ point;
            float maxRadius = 0;
            float roadWidth = 0;   //记录道路宽度
            float predictRadius = 0;     //记录预测半径
            float currAngle = i;
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
            if(maxRadius<5)continue;
            // if(left_curb.points.size()>1){
            //     pcl::PointXYZ point_tem = left_curb.back();
            //     roadWidth = abs(point_tem.y);
            //     predictRadius  = roadWidth / cos(currAngle+delt);
            //     int flag = maxRadius >= predictRadius ? 1 : -1;
            //     float diff = abs(maxRadius - predictRadius);
                // if(flag==1 && diff>1){//小到大的跳变，删除原本的
                //     left_curb.clear();

                // }
                // else if(flag==-1 && diff > 1){
                //     //大到小的跳变，删除后面的
                //     left_curb.points.push_back(point);
                //     break;
                // }
            //}
            
            left_curb.points.push_back(point);
        }
        left_curb = PointFilter(left_curb.makeShared(),2,0.5);
    }//提取道路边线

    else{   
        for(pcl::PointCloud<pcl::PointXYZ>::iterator it=cloud->begin();it!=cloud->end();++it){
            if(it->y>0){
                left_curb.push_back(*it);   //双车道仅保留左侧的道路中心线   
            }
            
        }
    }//提取车道线点

    //curbFit(left_curb,right_curb);  //拟合边线，输出点
    return right_curb + left_curb;

}

pcl::PointCloud<pcl::PointXYZ> hull_curb(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, bool state){

    /*
        该函数采用凸包围方法整体提取出边界点，但对于两侧边界点的筛选方法还有待改进
    */
    pcl::PointCloud<pcl::PointXYZ> left_curb;
    pcl::PointCloud<pcl::PointXYZ> right_curb;
    if(state){
        pcl::ConvexHull<pcl::PointXYZ> hull;                  
        hull.setInputCloud(cloud);                   
        hull.setDimension(2);

        std::vector<pcl::Vertices> polygons;                 
        pcl::PointCloud<pcl::PointXYZ>::Ptr surface_hull(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ> left_point;
        pcl::PointCloud<pcl::PointXYZ> right_point;
    
        hull.reconstruct(*surface_hull, polygons);  //筛选出最小凸包点
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
        
        //按照y轴方向的距离筛选连续的左右两侧边界点
        int len_l = left_point.size();
        int len_r = right_point.size();
        for(int i=0;i<len_l-1;i++){
            float abs_ = abs(left_point[i].y - left_point[i+1].y); //相邻点间的跳变
            if(abs_<0.6){   //测试经验设定的阈值
                int len = left_curb.size();
                if(len>0){
                    float tem = left_curb[len-1].y;
                    int dy = abs(tem - left_point[i].y);
                    if(dy>0.5){     //防止一段连续的跳变段
                        continue;
                    }
                }
                left_curb.points.push_back(left_point[i]);
                //cout<<left_point[i].y<<endl;
            }
            
        }
        //cout<<"left_curb"<<left_curb.size()<<endl;
        for(int i=0;i<len_r-1;i++){
            float abs_ = abs(right_point[i].y - right_point[i+1].y);
            if(abs_<0.6){
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
    }//提取道路边线

    else{
        for(pcl::PointCloud<pcl::PointXYZ>::iterator it=cloud->begin();it!=cloud->end();++it){
            if(it->y>0){
                left_curb.push_back(*it);   //双车道仅保留左侧的道路中心线   
            }
            
        }
    }//提取车道线点

    curbFit(left_curb,right_curb);
    return left_curb+right_curb;
}


void curbFit(pcl::PointCloud<pcl::PointXYZ>& left_curb, pcl::PointCloud<pcl::PointXYZ>& right_curb)
{

    /*
        利用ransac方法拟合边界点，然后根据方程取x坐标输出该直线上的坐标点作为基准点
        注意按需求设定输出点的长度和点之间的间距 （490行   513行）
    */
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients1(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers1(new pcl::PointIndices);
    pcl::SACSegmentation<pcl::PointXYZ> seg;
//cout<<"right_curb"<<right_curb.size()<<endl;
    if(left_curb.size()>2){
        seg.setOptimizeCoefficients(true);    
        seg.setModelType(pcl::SACMODEL_LINE);  
        seg.setMethodType(pcl::SAC_RANSAC);     
        seg.setDistanceThreshold(0.5);        
        seg.setInputCloud(left_curb.makeShared());               
        seg.segment(*inliers, *coefficients);
        float x0_l = coefficients->values[0];
        float y0_l = coefficients->values[1];
        float z0_l = coefficients->values[2];
        float a_l = coefficients->values[3];
        float b_l = coefficients->values[4];
        float c_l = coefficients->values[5];
        left_curb.clear();
        
    
        for(float x=5;x<=12;x+=0.5){    //按需求设定
            pcl::PointXYZ point;
            point.x = x;
            point.y = b_l*(x-x0_l)/a_l + y0_l;
            point.z = c_l*(x-x0_l)/a_l + z0_l;
            //cout<<point.x<<point.y<<point.z<<endl;
            left_curb.points.push_back(point);
        }
        
    }
    if(right_curb.size()>2){
        seg.setOptimizeCoefficients(true);    
        seg.setModelType(pcl::SACMODEL_LINE);  
        seg.setMethodType(pcl::SAC_RANSAC);     
        seg.setDistanceThreshold(0.5);        
        seg.setInputCloud(right_curb.makeShared());               
        seg.segment(*inliers1, *coefficients1);
        float x0_r = coefficients1->values[0];
        float y0_r = coefficients1->values[1];
        float z0_r = coefficients1->values[2];
        float a_r = coefficients1->values[3];
        float b_r = coefficients1->values[4];
        float c_r = coefficients1->values[5];
        right_curb.clear();
    
        for(float x=5;x<=12;x+=0.5){        //按需求设定
            pcl::PointXYZ point;
            point.x = x;
            point.y = b_r*(x-x0_r)/a_r + y0_r;
            point.z = c_r*(x-x0_r)/a_r + z0_r;
            //cout<<point.x<<point.y<<point.z<<endl;
            right_curb.points.push_back(point);
        }
        
    }
}

};


#endif