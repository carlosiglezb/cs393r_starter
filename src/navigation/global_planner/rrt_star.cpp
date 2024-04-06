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

struct Point {
    double x, y, cost;
    Point(double x = 0, double y = 0, double cost = 0) : x(x), y(y), cost(cost) {}
    double distance(const Point& other) const {
        return std::sqrt(std::pow(x - other.x, 2) + std::pow(y - other.y, 2));
    }
};

struct LineSegment {
    Point start, end;
    LineSegment(const Point& start, const Point& end) : start(start), end(end) {}
};

std::vector<LineSegment> obstacles;

void readObstaclesFromFile(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }
    std::string line;
    while (std::getline(file, line)) {
        std::istringstream iss(line);
        double ax, ay, bx, by;
        char comma;
        if (!(iss >> ax >> comma >> ay >> comma >> bx >> comma >> by)) {
            std::cerr << "Error parsing line: " << line << std::endl;
            continue;
        }
        obstacles.push_back(LineSegment(Point(ax, ay), Point(bx, by)));
    }
    if (obstacles.empty()) {
        std::cout << "No obstacles were loaded from the file." << std::endl;
    } else {
        std::cout << "Total obstacles loaded: " << obstacles.size() << std::endl;
    }
    file.close();
}

int orientation(const Point &p, const Point &q, const Point &r) {
    double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
    if (val == 0) return 0;
    return (val > 0) ? 1 : 2;
}

bool onSegment(const Point &p, const Point &q, const Point &r) {
    return q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
           q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y);
}

bool checkIntersection(const LineSegment &line1, const LineSegment &line2) {
    Point p1 = line1.start, q1 = line1.end, p2 = line2.start, q2 = line2.end;
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);
    if (o1 != o2 && o3 != o4) return true;
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;
    return false;
}

class RRTStar {
private:
    std::vector<Point> nodes;
    std::vector<int> parent;
    std::unordered_map<int, double> costs;
    Point start, end;
    double stepSize, goalRadius;
    int maxIterations;
    double minX, maxX, minY, maxY, minDistanceToObstacle;

public:
    RRTStar(const Point& start, const Point& end, double stepSize, double goalRadius, int maxIterations, double minX, double maxX, double minY, double maxY, double minDistanceToObstacle)
        : start(start), end(end), stepSize(stepSize), goalRadius(goalRadius), maxIterations(maxIterations), minX(minX), maxX(maxX), minY(minY), maxY(maxY), minDistanceToObstacle(minDistanceToObstacle) {
        nodes.push_back(start);
        parent.push_back(-1);
        costs[0] = 0;
    }

    Point getRandomPoint() {
        double x = minX + (std::rand() % static_cast<int>(maxX - minX + 1));
        double y = minY + (std::rand() % static_cast<int>(maxY - minY + 1));
        return Point(x, y);
    }

    int getNearestNodeIndex(const Point& point) {
        int nearestIdx = 0;
        double nearestDist = std::numeric_limits<double>::max();
        for (size_t i = 0; i < nodes.size(); i++) {
            double dist = nodes[i].distance(point);
            if (dist < nearestDist) {
                nearestDist = dist;
                nearestIdx = i;
            }
        }
        return nearestIdx;
    }

    std::vector<int> findNearbyNodes(const Point& point, double radius) {
        std::vector<int> nearbyNodes;
        for (size_t i = 0; i < nodes.size(); i++) {
            if (nodes[i].distance(point) <= radius) {
                nearbyNodes.push_back(i);
            }
        }
        return nearbyNodes;
    }

    double pointToLineSegmentDistance(const Point& p, const LineSegment& line) {
        double px = p.x - line.start.x;
        double py = p.y - line.start.y;
        double sx = line.end.x - line.start.x;
        double sy = line.end.y - line.start.y;
        double segmentLengthSquared = sx * sx + sy * sy;
        if (segmentLengthSquared == 0.0) return p.distance(line.start);
        double t = std::max(0.0, std::min(1.0, (px * sx + py * sy) / segmentLengthSquared));
        Point projection(line.start.x + t * sx, line.start.y + t * sy);
        return p.distance(projection);
    }


    // bool isPathFree(const Point& newStart, const Point& newEnd, double minDistanceToObstacle) {
    //     LineSegment newPath(newStart, newEnd);
    //     for (const auto& obstacle : obstacles) {
    //         if (checkIntersection(newPath, obstacle)) return false;

    //         // Check distance from the new path's start and end points to each obstacle's endpoints
    //         if (
    //             newStart.distance(obstacle.start) < minDistanceToObstacle ||
    //             newStart.distance(obstacle.end) < minDistanceToObstacle ||
    //             newEnd.distance(obstacle.start) < minDistanceToObstacle ||
    //             newEnd.distance(obstacle.end) < minDistanceToObstacle) {
    //             return false;
    //         }
    //     }
    //     return true;
    // }

    bool isPathFree(const Point& newStart, const Point& newEnd, double minDistanceToObstacle) {
        LineSegment newPath(newStart, newEnd);
        double closestObstacleEndpointDist = std::numeric_limits<double>::max();
        double closestDistToNewEnd = std::numeric_limits<double>::max();

        for (const auto& obstacle : obstacles) {
            if (checkIntersection(newPath, obstacle)) return false;

            // Calculate the distance from the obstacle's endpoints to the new path
            double distStartToPath = pointToLineSegmentDistance(obstacle.start, newPath);
            double distEndToPath = pointToLineSegmentDistance(obstacle.end, newPath);
            closestObstacleEndpointDist = std::min({closestObstacleEndpointDist, distStartToPath, distEndToPath});

            // Calculate the distance from the new endpoint to the obstacle line segment
            double distNewEndToObstacle = std::min(pointToLineSegmentDistance(newEnd, obstacle), pointToLineSegmentDistance(newStart, obstacle));
            closestDistToNewEnd = std::min(closestDistToNewEnd, distNewEndToObstacle);
        }

        // Check if the closest distances are greater than minDistanceToObstacle
        if (closestObstacleEndpointDist < minDistanceToObstacle || closestDistToNewEnd < minDistanceToObstacle) {
            return false;
        }

        return true;
    }



    void addNewNode(const Point& randomPoint) {
        int nearestIdx = getNearestNodeIndex(randomPoint);
        Point nearestNode = nodes[nearestIdx];

        double theta = std::atan2(randomPoint.y - nearestNode.y, randomPoint.x - nearestNode.x);
        Point newNode(nearestNode.x + stepSize * std::cos(theta), nearestNode.y + stepSize * std::sin(theta));

        if (!isPathFree(nearestNode, newNode, minDistanceToObstacle)) return;

        std::vector<int> nearbyNodes = findNearbyNodes(newNode, 2 * stepSize);
        chooseBestParent(newNode, nearbyNodes);
        int newNodeIndex = nodes.size() - 1;
        rewire(nearbyNodes, newNodeIndex);
    }

    void chooseBestParent(Point& newNode, std::vector<int>& nearbyNodes) {
        double minCost = std::numeric_limits<double>::infinity();
        int bestParentIdx = -1;

        for (int idx : nearbyNodes) {
            double tempCost = costs[idx] + nodes[idx].distance(newNode);
            if (tempCost < minCost && isPathFree(nodes[idx], newNode, minDistanceToObstacle)) {
                minCost = tempCost;
                bestParentIdx = idx;
            }
        }

        if (bestParentIdx != -1) {
            parent.push_back(bestParentIdx);
            newNode.cost = minCost;
            costs[nodes.size()] = minCost;
            nodes.push_back(newNode);
        }
    }

    void rewire(std::vector<int>& nearbyNodes, int newNodeIndex) {
        Point& newNode = nodes[newNodeIndex];

        for (int idx : nearbyNodes) {
            if (idx == parent[newNodeIndex]) continue;

            double tempCost = newNode.cost + newNode.distance(nodes[idx]);
            if (tempCost < costs[idx] && isPathFree(newNode, nodes[idx], minDistanceToObstacle)) {
                parent[idx] = newNodeIndex;
                costs[idx] = tempCost;
            }
        }
    }

    void saveToFile(const std::string& pathFile, const std::string& treeFile) {
        std::ofstream pathStream(pathFile);
        std::ofstream treeStream(treeFile);

        int currentIndex = nodes.size() - 1;
        while (currentIndex != -1) {
            pathStream << nodes[currentIndex].x << ", " << nodes[currentIndex].y << std::endl;
            currentIndex = parent[currentIndex];
        }

        for (size_t i = 1; i < nodes.size(); ++i) {
            Point& start = nodes[parent[i]];
            Point& end = nodes[i];
            treeStream << start.x << ", " << start.y << ", " << end.x << ", " << end.y << std::endl;
        }

        pathStream.close();
        treeStream.close();
    }

    void generate() {
        std::srand(static_cast<unsigned int>(std::time(0)));
        for (int i = 0; i < maxIterations; ++i) {
            Point randomPoint = getRandomPoint();
            addNewNode(randomPoint);
            if (nodes.back().distance(end) < goalRadius) {
                std::cout << "Goal reached." << std::endl;
                break;
            }
        }
        saveToFile("final_path.txt", "tree_structure.txt");
    }
};

int main() {
    readObstaclesFromFile("./amrl_maps/GDC3/GDC3.vectormap.txt");
    // Point start(-35.50, 19.00), end(40.25, 26.27);
    Point start(40.25, 26.27), end(-35.50, 19.00);
    // Point start(-38.172, 12.61), end(-9.5, 13.7);

    double stepSize = 1;
    double goalRadius = 1.0;
    double minDistanceToObstacle = 0.2;
    int maxIterations = 100000000;
    
    double minX = -45, maxX = 45, minY = -30, maxY = 30;

    RRTStar rrt(start, end, stepSize, goalRadius, maxIterations, minX, maxX, minY, maxY, minDistanceToObstacle);
    rrt.generate();

    return 0;
}
