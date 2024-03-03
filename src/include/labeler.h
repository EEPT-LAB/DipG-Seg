// Copyright (C) 2020  I. Bogoslavskyi, C. Stachniss
//
// Permission is hereby granted, free of charge, to any person obtaining a
// copy of this software and associated documentation files (the "Software"),
// to deal in the Software without restriction, including without limitation
// the rights to use, copy, modify, merge, publish, distribute, sublicense,
// and/or sell copies of the Software, and to permit persons to whom the
// Software is furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
// FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
// DEALINGS IN THE SOFTWARE.

/* --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
Due to the good implementation of BFS Labeler from the https://github.com/PRBonn/depth_clustering.git, our Laberler's code references it a lot. 
Thus, although our code is not a copy of the original code, we still want to thank the authors of the original code and keep the original copyright notice.
----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------*/
#pragma once
#include <vector>
#include <memory>
#include <list>
#include <opencv2/core.hpp>
#include <queue>

namespace Algorithm{
    class Labeler{
    public:
        std::vector<cv::Point2i> NEIGHBORS;
        Labeler(cv::Mat & input_image);
        ~Labeler();
        void label_connected_ground(cv::Mat & labeled_image);
        void BFS_laber(size_t label, const cv::Point2i & start_coord);
    
    private:
        cv::Mat input_image_;
        cv::Mat labeled_record_image_;
        std::shared_ptr<std::vector<cv::Point2i>> ground_pixels_ptr_;        
    };


    Labeler::Labeler(cv::Mat & input_image):input_image_(input_image)
    {
        labeled_record_image_ = cv::Mat::zeros(input_image_.size(), cv::DataType<uint8_t>::type);
        NEIGHBORS.reserve(4);
        NEIGHBORS.emplace_back(cv::Point2i(0,1));
        NEIGHBORS.emplace_back(cv::Point2i(0,-1));
        NEIGHBORS.emplace_back(cv::Point2i(1,0));
        NEIGHBORS.emplace_back(cv::Point2i(-1,0));
    }
    
    Labeler::~Labeler()
    {
    }

    void Labeler::label_connected_ground(cv::Mat & labeled_image){
        int r = input_image_.rows - 1;
        for(int c=0; c<input_image_.cols; ++c){ 
            if(labeled_record_image_.at<uint8_t>(r,c) > 0){
                continue;
            }
            BFS_laber(40, cv::Point2i(c,r));
        }
        labeled_image = labeled_record_image_.clone();
    }
    void Labeler::BFS_laber(size_t label, const cv::Point2i & start_coord){
        std::queue<cv::Point2i> label_queue;

        label_queue.push(start_coord);

        if(input_image_.at<uint8_t>(start_coord) != 0){
            return;
        }
        else{
            labeled_record_image_.at<uint8_t>(start_coord) = label;
        }

        while (!label_queue.empty())
        {
            const auto current_coord = label_queue.front();
            label_queue.pop();
            labeled_record_image_.at<uint8_t>(current_coord) = label;
            for(auto neighbor: NEIGHBORS){
                auto current_coord_temp = current_coord + neighbor;
                // rows
                if(current_coord_temp.y > input_image_.rows-1 || current_coord_temp.y <0){
                    continue;
                }
                //cols
                if(current_coord_temp.x >input_image_.cols -1){
                    current_coord_temp.x =current_coord_temp.x  - input_image_.cols;
                }
                else if(current_coord_temp.x <0){
                    current_coord_temp.x = input_image_.cols + current_coord_temp.x;
                }

                size_t label_neighbor = labeled_record_image_.at<uint8_t>(current_coord_temp);
                if(label_neighbor>0){
                    continue;
                }
                uint8_t not_ground = input_image_.at<uint8_t>(current_coord_temp);
                if(!not_ground){
                    labeled_record_image_.at<uint8_t>(current_coord_temp) = label;
                    label_queue.push(current_coord_temp);
                }
            }
        }
    }
}

