#include <iostream>
#include <pcl_conversions/pcl_conversions.h>
#include <boost/format.hpp>
/*
labels: 
  0 : "unlabeled"
  1 : "outlier"
  10: "car"
  11: "bicycle"
  13: "bus"
  15: "motorcycle"
  16: "on-rails"
  18: "truck"
  20: "other-vehicle"
  30: "person"
  31: "bicyclist"
  32: "motorcyclist"
  40: "road"
  44: "parking"
  48: "sidewalk"
  49: "other-ground"
  50: "building"
  51: "fence"
  52: "other-structure"
  60: "lane-marking"
  70: "vegetation"
  71: "trunk"
  72: "terrain"
  80: "pole"
  81: "traffic-sign"
  99: "other-object"
  252: "moving-car"
  253: "moving-bicyclist"
  254: "moving-person"
  255: "moving-motorcyclist"
  256: "moving-on-rails"
  257: "moving-bus"
  258: "moving-truck"
  259: "moving-other-vehicle"
*/
namespace Utils
{

class Evaluator {
public:
    Evaluator(std::string &seq, size_t frame_num, std::string &label_path) {
        abs_path_ = label_path;
        seq_num_ = seq;
        label_path_ = abs_path_ +seq+ "/labels";
        size_t label_num;
        for (label_num = 0; label_num<frame_num; label_num++) {
            std::string filename = (boost::format("%s/%06d.label") % label_path_ % label_num).str();
            if (!boost::filesystem::exists(filename)) {
                printf("The label file is LOST!!!:\n%s", filename.c_str());
                exit(1);
            }
        }
    }

    ~Evaluator() {}

    int evaluate_frame(size_t idx, std::vector<double> & result, bool print_flag, pcl::PointCloud<pcl::PointXYZI>& pcd_in, size_t * ground_idx) const {
        std::string   label_file = (boost::format("%s/%06d.label") % label_path_ % idx).str();
        std::ifstream label_stream(label_file, std::ios::binary);
        if (!label_stream.is_open()) {
            printf("\033[31;1mCANNOT OPEN LABEL FILE!\033[0m\n");
            return -1;
        }
        label_stream.seekg(0, std::ios::end);
        size_t num_points = label_stream.tellg()/ sizeof(uint32_t);

        label_stream.seekg(0, std::ios::beg);
        std::vector<uint32_t> labels(num_points);
        label_stream.read((char *) &labels[0], num_points * sizeof(uint32_t));
        if(num_points!=pcd_in.size()){
            printf("\n\n\nXXXXXXXXXXXXXXXXXXXXXXX---LENGTH ERROR: %ld != %ld---XXXXXXXXXXXXXXXXXXXXXXXX\n\n\n", num_points, pcd_in.size());
            exit(1);
        }
        double TN=0, TP=0, FP=0, FN=0;
        for (size_t i = 0; i < num_points; i++) {
            int label_temp = labels[i] & 0xFFFF;
            if(label_temp == 40 or label_temp == 44 or label_temp==48 or label_temp == 49 or label_temp==60 or label_temp==72){
                label_temp = 1;                
            }
            else{
                label_temp = 0;
            }
            if( ground_idx[i] == 1 and label_temp == 1){
                TP+=1;
            }
            else if( ground_idx[i] != 1 and label_temp != 1){
                TN+=1;
            }
            else if( ground_idx[i] != 1 and label_temp == 1){
                FN+=1;
            }
            else if( ground_idx[i] == 1 and label_temp != 1){
                FP+=1;
            }
        }
        // printf("------------------------------------->frame id: %d\n", idx);
        result.resize(4);
        result[0] = TN;
        result[1] = FN;
        result[2] = FP;
        result[3] = TP;
        if(print_flag){
            printf("\033[31mTN: %.0f  \033[0m ", TN);
            printf("\033[31mFN: %.0f  \033[0m ", FN);
            printf("\033[33mFP: %.0f  \033[0m ", FP);
            printf("\033[33mTP: %.0f  \033[0m ", TP);
            printf("\033[1;32m-->PRE: %.4f  \033[0m ", TP*100/(TP+FP));
            printf("\033[1;35mRECALL: %.4f  \033[0m\n",TP*100/(TP+FN));
        }
        return 0;
    }

private:
    std::string label_path_;
    std::string pc_path_;
    std::string pred_path_;
    std::string seq_num_;
    std::string abs_path_ ;
    std::string result_file_name_;
};

    
} // namespace Utils