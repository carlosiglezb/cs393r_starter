#ifndef NAV_UTILS_RRT_HPP
#define NAV_UTILS_RRT_HPP

#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <cstdlib>
#include <ctime>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <eigen3/Eigen/Dense>
#include "geometry_tools.hpp"

class RRT {

public:
    RRT(double stepSize=0.5,
        double goalRadius=0.5,
        int maxIterations=1e8,
        double minX=-45.,
        double maxX=45.,
        double minY=-30.,
        double maxY=30);
    ~RRT();
    void generate(const Point& start, const Point& end);
    void readObstaclesFromFile(const std::string& filename);
    const std::vector<Eigen::Vector2f> getRRTPathPoints();
    const Eigen::Vector2f getRRTPathPoint(const int idx);

private:
    Point getRandomPoint();
    int getNearestNodeIndex(const Point& point);
    bool isPathFree(const Point& start, const Point& end);
    bool addNewNode(const Point& randomPoint);
    void saveToFile(const std::string& pathFile, const std::string& treeFile);

private:
    std::vector<Point> nodes;
    std::vector<int> parent; // To store the tree structure
    Point start, end;
    double stepSize, goalRadius;
    int maxIterations;
    double minX, maxX, minY, maxY;
    std::vector<LineSegment> obstacles;
    std::vector<Eigen::Vector2f> rrt_path_points;
};


#endif //NAV_UTILS_RRT_HPP
