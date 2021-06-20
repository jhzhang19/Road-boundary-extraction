#include"include/Map_label.hpp"
#include"include/curbDetector.hpp"
#include<time.h>




int main(int argc, char* argv[]){
    if(argc!=2){
        /*
        执行时输入true/false
        true----车道检测图片的边界点提取
        false-----车道线检测图片的边界提取
        */
        cout<<"input param is wrong: \n eg:  ./map_label false/true"<<endl;
        cout<<"false: extract the points of the lane"<<endl;
        cout<<"true: extract the points of the road edge"<<endl;
        return -1;
    }
    bool state;
    istringstream(argv[1])>>boolalpha>>state;

    clock_t start,finish;
    clock_t start1, finish1;
    double totaltime, totaltime1;
    double static_time, average_time;
    double static_time1, average_time1;
    static_time1 = 0;
    static_time = 0;
    
   /*初始化处理对象*/
    Map_label processor;    //点云投影、ROI提取处理对象
    processor.initParams();
    curbDetector curb;      //边界点提取对象
    curb.initParam();
    
    
    
    /*道路检测、车道线检测二值化图片、点云pcd文件存储位置*/
    string path1 = curb.curbDetect_params.road_mask_img;   // road mask image path    
    string path2 = curb.curbDetect_params.pc_pcd;   //pcd path  
    string path3 = curb.curbDetect_params.lane_mask_img; //lane mask image path  
    string path4 = curb.curbDetect_params.original_rgb_img;// original RGB image  path
    string path5 = curb.curbDetect_params.res_vis_path; //project result and vis path
    string path6 = curb.curbDetect_params.road_pcd;     //result pcd
    string path7 = curb.curbDetect_params.lane_pcd;     //result pcd

    vector<string> file_name1;  //image_name vc
    string path_image = "";

    /*判断不同条件，读入不同文件*/
    if(state){
        processor.GetFileNames(path1, file_name1);  //读取道路分割图片
    }
    else{
        processor.GetFileNames(path3, file_name1);  //读取车道线分割图片
    }

    const int len = file_name1.size(); 
    for(int i = 0; i < len; i++)
    {
        
        /*注意此处要求点云文件名与图片文件名要对应相同，才能相匹配*/
        if(state){
            path_image =path1 + "/" + file_name1[i];
        }
        else{
            path_image =path3 + "/" + file_name1[i];
        }
        string path_rgb = path4 + "/" + file_name1[i];
        string file_name2 = file_name1[i].substr(0, file_name1[i].rfind(".")) + ".pcd";  //pcd file name
        std::string path_pcd = path2 + "/" + file_name2;
        cout<<"processing the:  "<<file_name1[i]<<" the "<<i+1<<" frame"<<endl;
        
        /*数据读取  */
        cv::Mat image_;
        cv::Mat image_RGB;
        pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ> output;
        image_ = cv::imread(path_image);
        image_RGB = cv::imread(path_rgb);
        if(pcl::io::loadPCDFile<pcl::PointXYZ>(path_pcd, *pc)==-1){
            cout<<"pointcloud name and path is error"<<endl;
            return -1;
        }

        /*调试代码
        //cout<<"get image and pointcloud"<<"pointcloud size "<< pc->size()<<endl;
        //string windowName = "First steps in OpenCV";
        // cv::namedWindow(windowName, 1); 
        // cv::imshow(windowName, image_lane);
        // cv::waitKey(0); 
        */

        /*投影、ROI截取、筛选出路面或者车道线点云*/
        start = clock();
        pcl::PointCloud<pcl::PointXYZ>::Ptr extract_pc(new pcl::PointCloud<pcl::PointXYZ>);
        extract_pc = processor.ProjectAndExtract(image_, pc);
        //processor.Visualization(extract_pc);  //提取的可视化点云
        extract_pc -> height = 1;
        extract_pc -> width = extract_pc->size();
        if(state){  //只有当提取的是路面点云是才滤波，若是车道线一般不存在离群杂点
            *extract_pc = curb.PointFilter(extract_pc, curb.curbDetect_params.k, curb.curbDetect_params.filter_r);
        }
        curb.cloud_out.clear();

        /*边界提取*/
        //1.分线束提取方法
        //2.极坐标整体提取
        //3.凸包围形式整体提取

        ///-----------------------------------------------------------------------------------///
        /*1.分线束提取*/

        // curb.cloud_in = *extract_pc;
        // curb.cloud_cleaned = curb.cleanPoints(curb.cloud_in);   //删除未选中的线束
        // curb.cloud_out = curb.detector(curb.cloud_cleaned,state);
       
       
       
       ///-----------------------------------------------------------------------------------///

       /*2.极坐标形式提取*/
       start1 = clock();
       curb.cloud_out = curb.polar_detect(extract_pc,state);
       finish1 = clock();
       finish = clock();

        totaltime = (double)(finish-start)/CLOCKS_PER_SEC;
        totaltime1 = (double)(finish1-start1)/CLOCKS_PER_SEC;
        static_time += totaltime;
        static_time1 += totaltime1;
        cout<<"全流程处理时间："<<totaltime<<" s"<<endl;
        cout<<"极坐标扫描输出时间："<<totaltime1<<" s"<<endl;
       ///-----------------------------------------------------------------------------------///
       /*3.凸包围形式提取*/
       //curb.cloud_out = curb.hull_curb(extract_pc, state);

       //投影输出查看结果,需要时解开注释
        string save_path = path5 + "/" + file_name1[i];
        //processor.ProjectAndVis(image_RGB, curb.cloud_out.makeShared(), false, save_path);
        //processor.ProjectAndVis(image_, extract_pc, false, save_path);
        // if(state){
            if(curb.cloud_out.size()>0){
                pcl::io::savePCDFileASCII(path6 + "/" + file_name1[i].substr(0, file_name1[i].rfind(".")) + ".pcd" , *extract_pc);
            }
            
        // }
        // else{
        //     if(curb.cloud_out.size()>0){
        //         pcl::io::savePCDFileASCII(path7 + "/" + file_name1[i].substr(0, file_name1[i].rfind(".")) + ".pcd" , curb.cloud_out);
        //     }
            
        // }
        



    }
    average_time = static_time / len;
    average_time1 = static_time1 /len;
    cout<<"平均时间（总）："<< average_time<<endl;
    cout<<"平均时间（扫描）："<<average_time1<<endl;
    return 0;
}


