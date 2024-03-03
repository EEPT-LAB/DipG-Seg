#include "time_utils.h"
#include "kitti_loader.h"
#include "evaluate.h"
#include "dipgseg.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

/*****************************************************************************************
This is the main function of offline evaluation on KITTI dataset.
For fast evaluating on all kitti sequences, you can refer to DipG-Seg/scripts/eval_on_kitti.py,
and follow the instructions at the begining of that file.
ALso, you can run this node as follows in your terminal:
```bash
roscore
source {the path of your catkin workspace}/devel/setup.bash
rosrun dipgseg offline_kitti_node 00
# where "00" is the sequence number of KITTI dataset.
```
Also, this node would publish the ground and non-ground pointclouds to the topic "/ground" and "/non_ground" respectively.
Thus, you can visualize the results in rviz by importing the rviz config file in DipG-Seg/rviz/reprj.rviz.
*****************************************************************************************/
using namespace KittiLoader;
int main(int argc, char** argv){

    ros::init(argc, argv, "dipgseg_offline_kitti_node");
    ros::NodeHandle nh;
    ros::Publisher ground_pub;
    ros::Publisher non_ground_pub;
    ros::Rate loop_rate(10);

    std::string sequence_num("-1");
    std::string output_path;
    size_t bin_file_num=0;
    std::vector<double> time_cost;
    std::vector<double> time_cost_all_alg;
    Utils::Evaluator * eval_ptr;
    double seq_eval_res[4]={0};
    bool eval_flag = false;
    bool verbose = false;
    std::string dataset_path_pre;

    ground_pub= nh.advertise<sensor_msgs::PointCloud2>("/ground", 10);
    non_ground_pub= nh.advertise<sensor_msgs::PointCloud2>("/non_ground", 10);
    DIPGSEG::Dipgseg dipgseg = DIPGSEG::Dipgseg();

    if(argc>1){
        sequence_num = argv[1];
    }
    else{
        printf("NO args are input!\n");
        return 0;
    }
    
    time_t t_now = time(0);
    char time_str[64];
    strftime(time_str, sizeof(time_str), "%Y-%m-%d-%H-%M-%S",localtime(&t_now) );
    printf("now time is----------*********: %s\n", &time_str[0]);

    nh.param<bool>("eval_flag", eval_flag, false);
    nh.param<bool>("verbose", verbose, false);
    nh.param<std::string>("dataset_path", dataset_path_pre, "NONE");
    nh.param<std::string>("output_path", output_path, "NONE");
    if(dataset_path_pre=="NONE"){
        ROS_ERROR("CANNOT READ THE DATASET!");
    }
    if(output_path=="NONE"){
        ROS_ERROR("OUTPUT_PATH IS NOT SET!");
    }
    auto dataset_path = dataset_path_pre + sequence_num + std::string("/velodyne");
    auto cloud_reader =  FolderReader(dataset_path, ".bin", FolderReader::Order::SORTED);
    static size_t count=0;
    bin_file_num = cloud_reader.GetAllFilePaths().size();
    Utils::Evaluator evaluator(sequence_num, bin_file_num, dataset_path_pre);
    eval_ptr = &evaluator;

    // start to evaluate
    for (auto path : cloud_reader.GetAllFilePaths()) {
        Utils::TimeUtils timer;
        pcl::PointCloud<pcl::PointXYZI> cloud;
        read_pcl_from_kitti(path, cloud);
        if (!ros::ok()){
            fprintf(stderr, "ROS is SHUTDOWN!\n");
            return 0;
        }
        pcl::PointCloud<pcl::PointXYZI> cloud_ground, cloud_non_ground;
        size_t ground_idx[200000]={0};

        dipgseg.segment_ground_eval(cloud, cloud_ground, cloud_non_ground, &ground_idx[0]);//-----------------------------------------
        double time_all = dipgseg.get_whole_time();
        double time_seg = dipgseg.get_seg_time();
        if(verbose)
            printf("-------------time_all: %f, time_seg: %f\n", time_all, time_seg);

        sensor_msgs::PointCloud2 ground_msg, non_ground_msg;

        pcl::toROSMsg(cloud_ground, ground_msg);
        ground_msg.header.frame_id = "laser_link";
        ground_msg.header.stamp = ros::Time::now();
        ground_pub.publish(ground_msg);

        pcl::toROSMsg(cloud_non_ground, non_ground_msg);
        non_ground_msg.header.frame_id = "laser_link";
        non_ground_msg.header.stamp = ros::Time::now();
        non_ground_pub.publish(non_ground_msg);

        // write for evaluation*********************************
        if(eval_flag){
            std::vector<double> eval_res;
            eval_res.resize(4,0);
            eval_ptr->evaluate_frame(count, eval_res, !eval_flag, cloud, &ground_idx[0]);
            
            for(int res_idx=0; res_idx<4; ++res_idx){
                seq_eval_res[res_idx]+=eval_res[res_idx];
            }
            time_cost.push_back(time_all);
            time_cost_all_alg.push_back(time_seg);

            if(count==bin_file_num-1){
                int unset = access(output_path.c_str(), F_OK);
                    if(unset)
                        unset = system((std::string("mkdir -p ") + output_path).c_str());
                    if(unset != 0)
                        exit(1);
                std::string result =  (boost::format("%s%02d.csv") % output_path %sequence_num).str();

                std::ofstream result_stream(result);
                result_stream << sequence_num<<",";
                result_stream.precision(12);
                result_stream<< seq_eval_res[0]<<","<< seq_eval_res[1]<<","<< seq_eval_res[2]<<","<< seq_eval_res[3];
                result_stream.close();

                //time record
                std::string time_file = (boost::format("%s%02d_time_cost.csv") % output_path %sequence_num).str(); 
                std::string time_all_alg_file = (boost::format("%s%02d_time_cost_alg.csv") % output_path %sequence_num).str(); 
                std::ofstream time_stream_all(time_file);
                std::ofstream time_stream_alg(time_all_alg_file);
                for (size_t i=1; i<time_cost.size();++i){
                    if(i==time_cost.size()-1){
                        time_stream_all<<time_cost[i];
                        time_stream_alg<<time_cost_all_alg[i];
                    }
                    else{
                        time_stream_all<<time_cost[i]<<"\n";
                        time_stream_alg<<time_cost_all_alg[i]<<"\n";
                    }
                }
            }
        }
        else{
            std::vector<double> eval_res_nouse;
            eval_ptr->evaluate_frame(count, eval_res_nouse, (!eval_flag)&verbose, cloud, &ground_idx[0]);
            loop_rate.sleep();
        }

        count++;
    }
    
}