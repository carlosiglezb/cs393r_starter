#ifndef NAV_UTILS_RRT_STAR_HPP
#define NAV_UTILS_RRT_STAR_HPP

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
#include <algorithm>
#include <unordered_map>
#include <eigen3/Eigen/Dense>
#include "geometry_tools.hpp"

class RRTStar {

public:
    RRTStar(double stepSize=0.5,
        double goalRadius=0.5,
        int maxIterations=1e8,
        double minX=-45.,
        double maxX=45.,
        double minY=-30.,
        double maxY=30,
        float minDistanceToObstacle=0.2);    // done
    ~RRTStar();   // done
    void generate(const Point& start, const Point& end);  // done
    void readObstaclesFromFile(const std::string& filename);  // done
    const std::vector<Eigen::Vector2f> getRRTPathPoints();  // done
    const Eigen::Vector2f getRRTPathPoint(const int idx);   // done

private:
    Point getRandomPoint();   // done
    int getNearestNodeIndex(const Point& point);  // done
    std::vector<int> findNearbyNodes(const Point& point, double radius);  // done
    double pointToLineSegmentDistance(const Point& p, const LineSegment& line);   //done
    bool isPathFree(const Point& newStart, const Point& newEnd, double minDistanceToObstacle);  //done
    void addNewNode(const Point& randomPoint);    // done
    void chooseBestParent(Point& newNode, std::vector<int>& nearbyNodes);   // done
    void rewire(std::vector<int>& nearbyNodes, int newNodeIndex);  // done
    void saveToFile(const std::string& pathFile, const std::string& treeFile);  // done

private:
    std::vector<Point> nodes;
    std::vector<int> parent;
    std::unordered_map<int, double> costs;
    Point start, end;
    double stepSize, goalRadius;
    int maxIterations;
    double minX, maxX, minY, maxY, minDistanceToObstacle;

    std::vector<LineSegment> obstacles;
    std::vector<Eigen::Vector2f> rrt_path_points;
};


#endif //NAV_UTILS_RRT_STAR_HPP
