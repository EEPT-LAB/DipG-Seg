#pragma once

#include<chrono>

namespace Utils{

class TimeUtils{
    public:
    TimeUtils();
    ~TimeUtils();
    double get_duration(void);

    private:
    std::chrono::high_resolution_clock::time_point t_;
};

TimeUtils::TimeUtils(){
    t_=std::chrono::high_resolution_clock::now();
}

TimeUtils::~TimeUtils(){};

double TimeUtils::get_duration(void){
    auto t2 = std::chrono::high_resolution_clock::now();
    std::chrono::duration<double, std::ratio<1, 1>> d(t2 - t_);
    return d.count();
}

}