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


struct Point {
    double x, y;
    Point(double x = 0, double y = 0) : x(x), y(y) {}
    double distance(const Point& other) const {
      return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
    }
};

struct LineSegment {
    Point start, end;
    LineSegment(const Point& start, const Point& end) : start(start), end(end) {}
};


class RRT {

public:
    RRT(const Point& start, const Point& end, double stepSize, double goalRadius, int maxIterations, double minX, double maxX, double minY, double maxY);
    ~RRT();
    void generate();
    void readObstaclesFromFile(const std::string& filename);

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

};


#endif //NAV_UTILS_RRT_HPP
