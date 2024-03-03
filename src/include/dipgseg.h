/*
 * Copyright (c) 2023, Hao Wen, Chunhua Liu
 *
 This file is the implementation of our paper: DipG-Seg: Fast and Accurate Double Image-Based Pixel-Wise Ground Segmentation
 
 If you use this code, please cite our paper:
 [1] Hao Wen, Senyi Liu, Yuxin Liu, Chunhua Liu, "DipG-Seg: Fast and Accurate Double Image-Based Pixel-Wise Ground Segmentation", 
 IEEE Transactions on Intelligent Transportation Systems, 2023

For commercial use, please contact the authors. Prof Chunhua Liu <chunliu@cityu.edu.hk>

 * DipG-Seg is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License 3.0 as published by
 * the Free Software Foundation.

 * DipG-Seg is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, 
   this list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.
3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from this
   software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

// this version is using the inline function version of the find_row_index and find_col_index, and without Eigen for speed up

#include "projection_param.h"
#include "labeler.h"
#include "second_level_repair.h"
#include <chrono>
#include <math.h>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>


namespace DIPGSEG{
typedef std::list<size_t> PixelMappingPoints;
using UnprjCol =  std::vector<PixelMappingPoints>;
using UnprjMatrix = std::vector<UnprjCol>;

class Dipgseg{
public:
    Dipgseg();
    Dipgseg(double upper_z_std_thr, double lower_z_std_thr, 
            double edge_z_hori_thr, double upper_slope_thr, 
            double lower_slope_thr, double upper_flatness_thr, 
            double lower_flatness_thr);
    ~Dipgseg();
    template<typename PointType>
    void segment_ground_eval(pcl::PointCloud<PointType>& cloud_in, 
                            pcl::PointCloud<PointType>& ground_pcd,
                            pcl::PointCloud<PointType>& non_ground_pcd,  
                            size_t* ground_idx);

    template<typename PointType>
    void segment_ground(pcl::PointCloud<PointType>& cloud_in, 
                        pcl::PointCloud<PointType>& ground_pcd,
                        pcl::PointCloud<PointType>& non_ground_pcd);
    
    double get_whole_time(void){ return time_total_;};
    double get_seg_time(void){ return time_seg_;};


private: 
    inline int32_t find_row_index(float angle);
    inline int32_t find_col_index(float angle);
    
    const cv::Mat kernel2M2 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(2, 2));
    const cv::Mat kernel3M3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));

    double upper_z_std_thr;
    double lower_z_std_thr;
    double edge_z_hori_thr;

    double upper_slope_thr;
    double lower_slope_thr;
    double upper_flatness_thr;
    double lower_flatness_thr;

    int pcount_ground_thr;
    int pcount_non_ground_thr;

    double time_total_;
    double time_seg_;

};//class Dipgseg

// for using the optimal parameters in the paper
Dipgseg::Dipgseg():upper_z_std_thr(0.2), lower_z_std_thr(0.03), 
                   edge_z_hori_thr(0.05), 
                   upper_slope_thr(0.577), lower_slope_thr(0.268), 
                   upper_flatness_thr(0.289), lower_flatness_thr(0.134), 
                   pcount_ground_thr(6), pcount_non_ground_thr(8){
}

// for using the parameters set by users
Dipgseg::Dipgseg(double upper_z_std_thr, double lower_z_std_thr, 
                 double edge_z_hori_thr, 
                 double upper_slope_thr, 
                 double lower_slope_thr, double upper_flatness_thr, 
                 double lower_flatness_thr):
                 upper_z_std_thr(upper_z_std_thr), lower_z_std_thr(lower_z_std_thr), 
                 edge_z_hori_thr(edge_z_hori_thr), 
                 upper_slope_thr(upper_slope_thr), lower_slope_thr(lower_slope_thr), 
                 upper_flatness_thr(upper_flatness_thr), lower_flatness_thr(lower_flatness_thr), 
                 pcount_ground_thr(6), pcount_non_ground_thr(8){
}

Dipgseg::~Dipgseg(){
}

inline int32_t Dipgseg::find_row_index(float angle){
    int32_t found = static_cast<int32_t>((unprj_row_angles_const[0] - angle)*reverse_step_row_angle);
    if(found > first_index){
        if(found < last_rows_index){
            auto pre_diff = unprj_row_angles_const[found] - angle;
            return pre_diff < half_row_angle_step ? found : found + 1;
        }
        else{
            return last_rows_index;
        }
    }
    else{
        return 0;
    }
}
inline int32_t Dipgseg::find_col_index(float angle){
    int32_t found = static_cast<int32_t>((angle - unprj_col_angles_const[0])*reverse_step_col_angle);
    if(found > 0){
        if(found < last_cols_index){
            auto pre_diff = angle - unprj_col_angles_const[found];
            return pre_diff < half_col_angle_step ? found : found + 1;
        }
        else{
            return last_cols_index;
        }
    }
    else{
        return 0;
    }
}

template<typename PointType>
void Dipgseg::segment_ground_eval(pcl::PointCloud<PointType>& cloud_in, 
                            pcl::PointCloud<PointType>& ground_pcd,
                            pcl::PointCloud<PointType>& non_ground_pcd,  
                            size_t* ground_idx)
{
    UnprjMatrix unprj_maxtix = UnprjMatrix(row_angles_size, UnprjCol(col_angles_size));
    cv::Mat possibility(row_angles_size, col_angles_size, cv::DataType<uchar>::type, cv::Scalar(0));
    cv::Mat img_z(row_angles_size, col_angles_size, cv::DataType<float>::type, cv::Scalar(0.000121f));
    cv::Mat img_d(row_angles_size, col_angles_size, cv::DataType<float>::type, cv::Scalar(0.000121f));
    ground_pcd.reserve(cloud_in.size());
    non_ground_pcd.reserve(cloud_in.size());
    std::chrono::high_resolution_clock::time_point total_timer = std::chrono::high_resolution_clock::now();

    
    // start projection
    for(size_t index=0; index<cloud_in.size(); ++index){
        const auto& point = cloud_in.points[index];
        float dist = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
        if(dist < 4.0f){
            if(std::fabs(point.y) < close_region_boundary_y and point.x < close_region_boundary_x_pos and point.x > close_region_boundary_x_neg){
                continue;
            }
        }
        float row_angle = std::asin(point.z/dist);
        float col_angle = std::atan2(point.y, point.x);
        int32_t row_index = find_row_index(row_angle);
        int32_t col_index = find_col_index(col_angle);

        unprj_maxtix[row_index][col_index].emplace_back(index);

        auto& current_z = img_z.at<float>(row_index, col_index);
        
        if(current_z!=0.000121f) continue;

        auto& current_d = img_d.at<float>(row_index, col_index);
        current_d = std::sqrt(point.x*point.x + point.y*point.y);
        current_z =  point.z+sensor_height; 
    }

    // first-level repair
    for(int c=0;c<img_z.cols;++c){
        for(int r=1;r<img_z.rows-1;++r){
            if(img_z.at<float>(r,c)==0.000121f and img_z.at<float>(r-1,c) != 0.000121f and img_z.at<float>(r+1,c) != 0.000121f)
                img_z.at<float>(r,c) = (img_z.at<float>(r+1,c)+img_z.at<float>(r-1,c))*0.5;
        }
        if(img_z.at<float>(img_z.rows-1,c) == 0.000121f and img_z.at<float>(img_z.rows-2,c) != 0.000121f ){
            img_z.at<float>(img_z.rows-1,c) =img_z.at<float>(img_z.rows-2,c);
        }
        if(img_z.at<float>(0,c) == 0.000121f and img_z.at<float>(1,c) != 0.000121f ){
            img_z.at<float>(0,c) =img_z.at<float>(1,c);
        }
    }

    for(int c=0;c<img_d.cols;++c){
        for(int r=1;r<img_d.rows-1;++r){
            if(img_d.at<float>(r,c)==0.000121f and img_d.at<float>(r-1,c) != 0.000121f and img_d.at<float>(r+1,c) != 0.000121f)
                img_d.at<float>(r,c) = (img_d.at<float>(r+1,c)+img_d.at<float>(r-1,c))*0.5;
        }
        if(img_d.at<float>(img_d.rows-1,c) == 0.000121f and img_d.at<float>(img_d.rows-2,c) != 0.000121f ){
            img_d.at<float>(img_d.rows-1,c) =img_d.at<float>(img_d.rows-2,c);
        }
         if(img_d.at<float>(0,c) == 0.000121f and img_d.at<float>(1,c) != 0.000121f ){
            img_d.at<float>(0,c) =img_d.at<float>(1,c);
        }
    }

    const cv::Mat& repaired_img_z = second_level_repair(img_z, 5, 1.0f);
    const cv::Mat& repaired_img_d = second_level_repair(img_d, 5, 5.0f);
    cv::boxFilter(repaired_img_z, repaired_img_z, -1, cv::Size(3,3));

    std::chrono::high_resolution_clock::time_point seg_timer = std::chrono::high_resolution_clock::now();
    // 1. get the STDZ 
    cv::Mat mat_z_square = repaired_img_z.mul(repaired_img_z);
    cv::Mat kernel_var(3,3, cv::DataType<float>::type, cv::Scalar::all(0.125));
    cv::Mat kernel_mean(3,3, cv::DataType<float>::type, cv::Scalar::all(0.111111));
    cv::Mat mat_z_mean;
    cv::Mat mat_z_square_mean;
    cv::filter2D(mat_z_square, mat_z_square_mean, -1, kernel_var, cv::Point2i(-1,-1),0.0,cv::BORDER_REPLICATE);
    cv::filter2D(repaired_img_z, mat_z_mean, -1, kernel_mean,  cv::Point2i(-1,-1),0.0,cv::BORDER_REPLICATE);
    cv::Mat var = mat_z_square_mean - mat_z_mean.mul(mat_z_mean);
    cv::sqrt(var, var);

    // 2 edge
    cv::Mat edge_z_hori(repaired_img_z.size(), cv::DataType<float>::type, cv::Scalar::all(0.000121));
    for(int r=img_z.rows-1; r>1; r--){
        for(int c=0; c<img_z.cols; ++c){
            int j = 1;
            if(repaired_img_z.at<float>(r,c)==0.000121f){
                continue;
            }
            if(j+c>img_z.cols-1){
                edge_z_hori.at<float>(r,c) = fabs(repaired_img_z.at<float>(r,0) - repaired_img_z.at<float>(r,c));
                continue;
            }
            edge_z_hori.at<float>(r,c) = fabs(repaired_img_z.at<float>(r,c+j) - repaired_img_z.at<float>(r,c));
        }
    }

    //3. slope
    cv::Mat z_vertical(repaired_img_z.size(), cv::DataType<float>::type, cv::Scalar::all(0.0));
    for(int c=0; c<img_z.cols; ++c){
        for(int r=img_z.rows-1; r>0; r--){
            auto& j = cpst[r];
            if(repaired_img_z.at<float>(r,c)==0.000121f){
                continue;
            }
            if(r-j<0){
                z_vertical.at<float>(r-1,c) = z_vertical.at<float>(r-j+1,c);
                continue;
            }
            z_vertical.at<float>(r-1,c) = fabs(repaired_img_z.at<float>(r-j,c) - repaired_img_z.at<float>(r,c));
        }
    }

    cv::Mat d_vertical(repaired_img_z.size(), cv::DataType<float>::type, cv::Scalar::all(0.000121));
    for(int c=0; c<img_z.cols; ++c){
        for(int r=img_z.rows-1; r>0; r--){
            auto& j = cpst[r];
            if(repaired_img_d.at<float>(r,c)==0.000121f){
                d_vertical.at<float>(r-1,c) = cpst_d_th;
                continue;
            }
            if(r-j<0){
                d_vertical.at<float>(r-1,c) = d_vertical.at<float>(r-j+1,c);
                continue;
            }
            d_vertical.at<float>(r-1,c) = fabs(repaired_img_d.at<float>(r-j,c) - repaired_img_d.at<float>(r,c))+0.001;
        }
    }

    cv::Mat slope = z_vertical/d_vertical;

    // 4. slope_deviation
    cv::Mat slope_deviation_raw = cv::Mat(repaired_img_z.size(), cv::DataType<float>::type, cv::Scalar(0.0));
    for(int r=img_z.rows-1; r>1; r--){
        for(int c=0; c<img_z.cols; ++c){
            int j = 1;
            if(slope.at<float>(r,c)==0.001f){
                continue;
            }
            if(j+c>img_z.cols-1){
                continue;
            }
            slope_deviation_raw.at<float>(r,c) = fabs(slope.at<float>(r,c+j) - slope.at<float>(r,c));
        }
    }
   
   //5. smooth
    cv::boxFilter(slope_deviation_raw, slope_deviation_raw,-1, cv::Size(3,3));
    cv::boxFilter(slope, slope, -1, cv::Size(3,3));

    //6. pre-segmenting
    for(int r=img_z.rows-1; r>0; r--){
        for(int c=0; c<img_z.cols; ++c){
            float & p_edge_z_hori = edge_z_hori.at<float>(r,c);
            float & p_slope_deviation = slope_deviation_raw.at<float>(r,c);
            float & p_slope = slope.at<float>(r,c);
            if(p_edge_z_hori < edge_z_hori_thr){
                if(p_slope<lower_slope_thr or p_slope_deviation <lower_flatness_thr or var.at<float>(r,c)<lower_z_std_thr){
                    continue;
                }
                else{
                    possibility.at<uchar>(r,c) = 1;
                }
            }
            else{
                if (p_slope>upper_slope_thr or p_slope_deviation >upper_flatness_thr or var.at<float>(r,c)>upper_z_std_thr){
                    possibility.at<uchar>(r,c) = 1;
                }
            }
        }
    }

    // 7. fine segmentation
    //closing filter
    cv::morphologyEx(possibility,possibility,cv::MORPH_CLOSE, kernel3M3,cv::Point(-1, -1), 1,cv::BORDER_REPLICATE);
    //convolution based on MVK
    cv::Mat P_window = cv::Mat::ones(3,3,cv::DataType<float>::type);
    cv::Mat P_count;
    cv::filter2D(possibility, P_count, -1, P_window, cv::Point2i(-1,-1),0,cv::BORDER_REPLICATE);

    cv::Mat ground_pixel;
    cv::Mat non_ground_pixel;
    cv::compare(P_count, pcount_ground_thr, ground_pixel, cv::CMP_GT);
    cv::compare(P_count, pcount_non_ground_thr, non_ground_pixel, cv::CMP_GT);
    //opening filter    
    cv::morphologyEx(ground_pixel,ground_pixel,cv::MORPH_OPEN, kernel2M2,cv::Point(-1, -1), 1,cv::BORDER_REPLICATE);
    //superimpose
    possibility = (possibility & ground_pixel) | non_ground_pixel;   


    // 8. Ground labeler
    Algorithm::Labeler labeler(possibility);
    cv::Mat labeled_img;
    labeler.label_connected_ground(labeled_img);

    std::chrono::high_resolution_clock::time_point seg_timer_stop = std::chrono::high_resolution_clock::now();
    time_seg_ = std::chrono::duration<double, std::ratio<1, 1000>>(seg_timer_stop - seg_timer).count();  //unit ms

    for(int r=0; r<labeled_img.rows; ++r){
        for(int c=0; c<labeled_img.cols; ++c){
            if(labeled_img.at<uchar>(r,c)==40){
                for(auto &point_index: unprj_maxtix[r][c] ){
                    ground_pcd.push_back(cloud_in.points[point_index]);
                }
            }
            else{
                for(auto &point_index: unprj_maxtix[r][c] ){
                    non_ground_pcd.push_back(cloud_in.points[point_index]);
                }
            }
        }
    }
    std::chrono::high_resolution_clock::time_point total_timer_stop = std::chrono::high_resolution_clock::now();
    time_total_ = std::chrono::duration<double, std::ratio<1, 1000>>(total_timer_stop - total_timer).count();  //unit ms

    // You need to provide the ground_idx, which can be initialized as: size_t ground_idx[200000]={0};
    // Then, ground_idx will be filled with 1 for ground points and 0 for non-ground points.
    for(int r=0; r<labeled_img.rows; ++r){
        for(int c=0; c<labeled_img.cols; ++c){
            if(labeled_img.at<uchar>(r,c)==40){
                for(auto &point_index: unprj_maxtix[r][c] ){
                    ground_idx[point_index] = 1;
                }
            }
        }
    }
}


template<typename PointType>
void Dipgseg::segment_ground(pcl::PointCloud<PointType>& cloud_in, 
                            pcl::PointCloud<PointType>& ground_pcd,
                            pcl::PointCloud<PointType>& non_ground_pcd)
{
    UnprjMatrix unprj_maxtix = UnprjMatrix(row_angles_size, UnprjCol(col_angles_size));
    cv::Mat possibility(row_angles_size, col_angles_size, cv::DataType<uchar>::type, cv::Scalar(0));
    cv::Mat img_z(row_angles_size, col_angles_size, cv::DataType<float>::type, cv::Scalar(0.000121f));
    cv::Mat img_d(row_angles_size, col_angles_size, cv::DataType<float>::type, cv::Scalar(0.000121f));
    ground_pcd.reserve(cloud_in.size());
    non_ground_pcd.reserve(cloud_in.size());
    std::chrono::high_resolution_clock::time_point total_timer = std::chrono::high_resolution_clock::now();

    
    // start projection
    for(size_t index=0; index<cloud_in.size(); ++index){
        const auto& point = cloud_in.points[index];
        float dist = sqrt(point.x*point.x + point.y*point.y + point.z*point.z);
        if(dist < 4.0f){
            if(std::fabs(point.y) < close_region_boundary_y and point.x < close_region_boundary_x_pos and point.x > close_region_boundary_x_neg){
                continue;
            }
        }
        float row_angle = std::asin(point.z/dist);
        float col_angle = std::atan2(point.y, point.x);
        int32_t row_index = find_row_index(row_angle);
        int32_t col_index = find_col_index(col_angle);

        unprj_maxtix[row_index][col_index].emplace_back(index);

        auto& current_z = img_z.at<float>(row_index, col_index);
        
        if(current_z!=0.000121f) continue;

        auto& current_d = img_d.at<float>(row_index, col_index);
        current_d = std::sqrt(point.x*point.x + point.y*point.y);
        current_z =  point.z+sensor_height; 
    }

    // first-level repair
    for(int c=0;c<img_z.cols;++c){
        for(int r=1;r<img_z.rows-1;++r){
            if(img_z.at<float>(r,c)==0.000121f and img_z.at<float>(r-1,c) != 0.000121f and img_z.at<float>(r+1,c) != 0.000121f)
                img_z.at<float>(r,c) = (img_z.at<float>(r+1,c)+img_z.at<float>(r-1,c))*0.5;
        }
        if(img_z.at<float>(img_z.rows-1,c) == 0.000121f and img_z.at<float>(img_z.rows-2,c) != 0.000121f ){
            img_z.at<float>(img_z.rows-1,c) =img_z.at<float>(img_z.rows-2,c);
        }
        if(img_z.at<float>(0,c) == 0.000121f and img_z.at<float>(1,c) != 0.000121f ){
            img_z.at<float>(0,c) =img_z.at<float>(1,c);
        }
    }

    for(int c=0;c<img_d.cols;++c){
        for(int r=1;r<img_d.rows-1;++r){
            if(img_d.at<float>(r,c)==0.000121f and img_d.at<float>(r-1,c) != 0.000121f and img_d.at<float>(r+1,c) != 0.000121f)
                img_d.at<float>(r,c) = (img_d.at<float>(r+1,c)+img_d.at<float>(r-1,c))*0.5;
        }
        if(img_d.at<float>(img_d.rows-1,c) == 0.000121f and img_d.at<float>(img_d.rows-2,c) != 0.000121f ){
            img_d.at<float>(img_d.rows-1,c) =img_d.at<float>(img_d.rows-2,c);
        }
         if(img_d.at<float>(0,c) == 0.000121f and img_d.at<float>(1,c) != 0.000121f ){
            img_d.at<float>(0,c) =img_d.at<float>(1,c);
        }
    }

    const cv::Mat& repaired_img_z = second_level_repair(img_z, 5, 1.0f);
    const cv::Mat& repaired_img_d = second_level_repair(img_d, 5, 5.0f);
    cv::boxFilter(repaired_img_z, repaired_img_z, -1, cv::Size(3,3));

    std::chrono::high_resolution_clock::time_point seg_timer = std::chrono::high_resolution_clock::now();
    // 1. get the STDZ 
    cv::Mat mat_z_square = repaired_img_z.mul(repaired_img_z);
    cv::Mat kernel_var(3,3, cv::DataType<float>::type, cv::Scalar::all(0.125));
    cv::Mat kernel_mean(3,3, cv::DataType<float>::type, cv::Scalar::all(0.111111));
    cv::Mat mat_z_mean;
    cv::Mat mat_z_square_mean;
    cv::filter2D(mat_z_square, mat_z_square_mean, -1, kernel_var, cv::Point2i(-1,-1),0.0,cv::BORDER_REPLICATE);
    cv::filter2D(repaired_img_z, mat_z_mean, -1, kernel_mean,  cv::Point2i(-1,-1),0.0,cv::BORDER_REPLICATE);
    cv::Mat var = mat_z_square_mean - mat_z_mean.mul(mat_z_mean);
    cv::sqrt(var, var);

    // 2 edge
    cv::Mat edge_z_hori(repaired_img_z.size(), cv::DataType<float>::type, cv::Scalar::all(0.000121));
    for(int r=img_z.rows-1; r>1; r--){
        for(int c=0; c<img_z.cols; ++c){
            int j = 1;
            if(repaired_img_z.at<float>(r,c)==0.000121f){
                continue;
            }
            if(j+c>img_z.cols-1){
                edge_z_hori.at<float>(r,c) = fabs(repaired_img_z.at<float>(r,0) - repaired_img_z.at<float>(r,c));
                continue;
            }
            edge_z_hori.at<float>(r,c) = fabs(repaired_img_z.at<float>(r,c+j) - repaired_img_z.at<float>(r,c));
        }
    }

    //3. slope
    cv::Mat z_vertical(repaired_img_z.size(), cv::DataType<float>::type, cv::Scalar::all(0.0));
    for(int c=0; c<img_z.cols; ++c){
        for(int r=img_z.rows-1; r>0; r--){
            auto& j = cpst[r];
            if(repaired_img_z.at<float>(r,c)==0.000121f){
                continue;
            }
            if(r-j<0){
                z_vertical.at<float>(r-1,c) = z_vertical.at<float>(r-j+1,c);
                continue;
            }
            z_vertical.at<float>(r-1,c) = fabs(repaired_img_z.at<float>(r-j,c) - repaired_img_z.at<float>(r,c));
        }
    }

    cv::Mat d_vertical(repaired_img_z.size(), cv::DataType<float>::type, cv::Scalar::all(0.000121));
    for(int c=0; c<img_z.cols; ++c){
        for(int r=img_z.rows-1; r>0; r--){
            auto& j = cpst[r];
            if(repaired_img_d.at<float>(r,c)==0.000121f){
                d_vertical.at<float>(r-1,c) = cpst_d_th;
                continue;
            }
            if(r-j<0){
                d_vertical.at<float>(r-1,c) = d_vertical.at<float>(r-j+1,c);
                continue;
            }
            d_vertical.at<float>(r-1,c) = fabs(repaired_img_d.at<float>(r-j,c) - repaired_img_d.at<float>(r,c))+0.001;
        }
    }

    cv::Mat slope = z_vertical/d_vertical;

    // 4. slope_deviation
    cv::Mat slope_deviation_raw = cv::Mat(repaired_img_z.size(), cv::DataType<float>::type, cv::Scalar(0.0));
    for(int r=img_z.rows-1; r>1; r--){
        for(int c=0; c<img_z.cols; ++c){
            int j = 1;
            if(slope.at<float>(r,c)==0.001f){
                continue;
            }
            if(j+c>img_z.cols-1){
                continue;
            }
            slope_deviation_raw.at<float>(r,c) = fabs(slope.at<float>(r,c+j) - slope.at<float>(r,c));
        }
    }
   
   //5. smooth
    cv::boxFilter(slope_deviation_raw, slope_deviation_raw,-1, cv::Size(3,3));
    cv::boxFilter(slope, slope, -1, cv::Size(3,3));

    //6. pre-segmenting
    for(int r=img_z.rows-1; r>0; r--){
        for(int c=0; c<img_z.cols; ++c){
            float & p_edge_z_hori = edge_z_hori.at<float>(r,c);
            float & p_slope_deviation = slope_deviation_raw.at<float>(r,c);
            float & p_slope = slope.at<float>(r,c);
            if(p_edge_z_hori < edge_z_hori_thr){
                if(p_slope<lower_slope_thr or p_slope_deviation <lower_flatness_thr or var.at<float>(r,c)<lower_z_std_thr){
                    continue;
                }
                else{
                    possibility.at<uchar>(r,c) = 1;
                }
            }
            else{
                if (p_slope>upper_slope_thr or p_slope_deviation >upper_flatness_thr or var.at<float>(r,c)>upper_z_std_thr){
                    possibility.at<uchar>(r,c) = 1;
                }
            }
        }
    }

    // 7. fine segmentation
    //closing filter
    cv::morphologyEx(possibility,possibility,cv::MORPH_CLOSE, kernel3M3,cv::Point(-1, -1), 1,cv::BORDER_REPLICATE);
    //convolution based on MVK
    cv::Mat P_window = cv::Mat::ones(3,3,cv::DataType<float>::type);
    cv::Mat P_count;
    cv::filter2D(possibility, P_count, -1, P_window, cv::Point2i(-1,-1),0,cv::BORDER_REPLICATE);

    cv::Mat ground_pixel;
    cv::Mat non_ground_pixel;
    cv::compare(P_count, pcount_ground_thr, ground_pixel, cv::CMP_GT);
    cv::compare(P_count, pcount_non_ground_thr, non_ground_pixel, cv::CMP_GT);
    //opening filter    
    cv::morphologyEx(ground_pixel,ground_pixel,cv::MORPH_OPEN, kernel2M2,cv::Point(-1, -1), 1,cv::BORDER_REPLICATE);
    //superimpose
    possibility = (possibility & ground_pixel) | non_ground_pixel;   


    // 8. Ground labeler
    Algorithm::Labeler labeler(possibility);
    cv::Mat labeled_img;
    labeler.label_connected_ground(labeled_img);

    std::chrono::high_resolution_clock::time_point seg_timer_stop = std::chrono::high_resolution_clock::now();
    time_seg_ = std::chrono::duration<double, std::ratio<1, 1000>>(seg_timer_stop - seg_timer).count();  //unit ms

    for(int r=0; r<labeled_img.rows; ++r){
        for(int c=0; c<labeled_img.cols; ++c){
            if(labeled_img.at<uchar>(r,c)==40){
                for(auto &point_index: unprj_maxtix[r][c] ){
                    ground_pcd.push_back(cloud_in.points[point_index]);
                }
            }
            else{
                for(auto &point_index: unprj_maxtix[r][c] ){
                    non_ground_pcd.push_back(cloud_in.points[point_index]);
                }
            }
        }
    }
    std::chrono::high_resolution_clock::time_point total_timer_stop = std::chrono::high_resolution_clock::now();
    time_total_ = std::chrono::duration<double, std::ratio<1, 1000>>(total_timer_stop - total_timer).count();  //unit ms
}

}//namespace DIPGSEG
