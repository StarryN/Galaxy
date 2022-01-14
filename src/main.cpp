#include <iostream>

#include "CorridorBuilder2d.hpp"

int main() {
    int mapsize = 1000;
    cv::Mat map(mapsize,mapsize,CV_8UC1, 255);

    int num = 1000;
    float bound = 400.0;
    std::default_random_engine random(time(NULL));
    std::uniform_real_distribution<float> r(50, 950);
    std::vector<Eigen::Vector2f> data;

    int loop_n = 0;
    float fx1 = mapsize/2-bound/2.0;
    float fy1 = mapsize/2-bound/2.0;
    float fx2 = mapsize/2+bound/2.0;
    float fy2 = mapsize/2+bound/2.0;
    while (loop_n < num) {
        float ax = r(random);
        float ay = r(random);
        if (ax < fx1 || ax > fx2 || ay < fy1 || ay > fy2) {
            data.push_back(Eigen::Vector2f(ax,ay));
            cv::circle(map,cv::Point2f(ax,mapsize-ay),3,0,-1);
            loop_n++;
        }
    }

    std::vector<Eigen::Vector3f> constrains; // a x + b y <= c;
    std::vector<cv::Point2f> vertexs;
    corridorBuilder2d(500,500,1500,data, vertexs, constrains);
    
    std::cout << constrains.size() << std::endl;
    std::cout << vertexs.size() << std::endl;

    for (size_t i=0; i<constrains.size(); i++) {
        float p1y = (constrains[i](2) - constrains[i](0)*200)/constrains[i](1);
        float p2y = (constrains[i](2) - constrains[i](0)*800)/constrains[i](1);
        cv::Point2d p1(200, mapsize-p1y);
        cv::Point2d p2(800, mapsize-p2y);
        cv::line(map, p1, p2, 100, 2);
    }

    for (size_t i=0; i<vertexs.size(); i++) {
        int ip1 = (i+1)%vertexs.size();
        cv::Point2f p1(vertexs[i].x, mapsize-vertexs[i].y);
        cv::Point2f p2(vertexs[ip1].x, mapsize-vertexs[ip1].y);
        cv::line(map, p1, p2, 50, 3);
    }

    cv::imshow("map",map);
    cv::waitKey(0);

    return 0;
}

