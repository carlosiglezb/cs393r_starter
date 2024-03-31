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
        char comma; // Used to skip commas

        if (!(iss >> ax >> comma >> ay >> comma >> bx >> comma >> by)) {
            std::cerr << "Error parsing line: " << line << std::endl;
            continue; // Skip malformed lines
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
    if (val == 0) return 0;  // colinear
    return (val > 0) ? 1 : 2; // clock or counterclock wise
}

bool onSegment(const Point &p, const Point &q, const Point &r) {
    return q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
           q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y);
}

bool checkIntersection(const LineSegment &line1, const LineSegment &line2) {
    // Log the line segments being checked
    // std::cout << "Checking intersection between: "
    //           << "(" << line1.start.x << ", " << line1.start.y << ") to (" << line1.end.x << ", " << line1.end.y << ") and "
    //           << "(" << line2.start.x << ", " << line2.start.y << ") to (" << line2.end.x << ", " << line2.end.y << ")" << std::endl;
    
    Point p1 = line1.start, q1 = line1.end, p2 = line2.start, q2 = line2.end;
    int o1 = orientation(p1, q1, p2);
    int o2 = orientation(p1, q1, q2);
    int o3 = orientation(p2, q2, p1);
    int o4 = orientation(p2, q2, q1);

    // General case
    if (o1 != o2 && o3 != o4)
        return true;

    // Special Cases
    // p1, q1 and p2 are colinear and p2 lies on segment p1q1
    if (o1 == 0 && onSegment(p1, p2, q1)) return true;

    // p1, q1 and p2 are colinear and q2 lies on segment p1q1
    if (o2 == 0 && onSegment(p1, q2, q1)) return true;

    // p2, q2 and p1 are colinear and p1 lies on segment p2q2
    if (o3 == 0 && onSegment(p2, p1, q2)) return true;

    // p2, q2 and q1 are colinear and q1 lies on segment p2q2
    if (o4 == 0 && onSegment(p2, q1, q2)) return true;

    return false; // Doesn't fall in any of the above cases
}


class RRT {
private:
    std::vector<Point> nodes;
    std::vector<int> parent; // To store the tree structure
    Point start, end;
    double stepSize, goalRadius;
    int maxIterations;
    double minX, maxX, minY, maxY;

public:
    RRT(const Point& start, const Point& end, double stepSize, double goalRadius, int maxIterations, double minX, double maxX, double minY, double maxY)
        : start(start), end(end), stepSize(stepSize), goalRadius(goalRadius), maxIterations(maxIterations), minX(minX), maxX(maxX), minY(minY), maxY(maxY) {
        nodes.push_back(start);
        parent.push_back(-1); // Start node has no parent
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

    bool isPathFree(const Point& start, const Point& end) {
        LineSegment newPath(start, end);
        // printf("Checking if path is free\n");
        for (const auto& obstacle : obstacles) {
            // printf("Checking intersection\n");
            if (checkIntersection(newPath, obstacle)) {
                // printf("Collision detected\n");
                return false;
            }
        }
        return true;
        }

    bool addNewNode(const Point& randomPoint) {
        // printf("Adding new node is running. \n")
        int nearestIdx = getNearestNodeIndex(randomPoint);
        Point nearestNode = nodes[nearestIdx];

        double theta = std::atan2(randomPoint.y - nearestNode.y, randomPoint.x - nearestNode.x);
        Point newNode(nearestNode.x + stepSize * std::cos(theta), nearestNode.y + stepSize * std::sin(theta));

        if (!isPathFree(nearestNode, newNode)) {
        // If path is not free, do not add the node and return false
        return false;
        }
        // if (newNode.x < minX || newNode.x > maxX || newNode.y < minY || newNode.y > maxY || !isPathFree(nearestNode, newNode)) {
        //     return false;
        // }

        nodes.push_back(newNode);
        parent.push_back(nearestIdx); // Store parent of newNode
        return newNode.distance(end) < goalRadius;
    }
    
    void saveToFile(const std::string& pathFile, const std::string& treeFile) {
        std::ofstream pathStream(pathFile);
        std::ofstream treeStream(treeFile);

        // Save final path
        int currentIndex = nodes.size() - 1;
        while (currentIndex != -1) {
            pathStream << nodes[currentIndex].x << ", " << nodes[currentIndex].y << std::endl;
            currentIndex = parent[currentIndex];
        }

        // Save tree structure
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
            if (addNewNode(randomPoint)) {
                std::cout << "Goal reached." << std::endl;
                break;
            }
        }

        saveToFile("final_path.txt", "tree_structure.txt");
    }
};

int main() {
    readObstaclesFromFile("../../../amrl_maps/GDC3/GDC3.vectormap.txt"); // You may need to change the path to the file
    double minX = -45, maxX = 45, minY = -30, maxY = 30; // This is measured from the GDC3.vectormap

    // Case 1 (easy)
    Point start(-41.70, 15.80), end(-13.95, 13.67);

    // Case 2 (complicated)
    // Point start(-40.25, -25.95), end(40.25, 26.27);

    // random vector map
    // readObstaclesFromFile("/Users/zhiyunjerrydeng/cs393r_starter/src/navigation/dengzy_checkpoint1/map_data.txt");
    // std::cout << "Obstacles loaded: " << obstacles.size() << std::endl;
    // Point start(-25, -25), end(25, 25);
    // double minX = -30, maxX = 30, minY = -30, maxY = 30;

    double stepSize = 0.5;
    double goalRadius = 0.5;
    int maxIterations = 100000000;
    


    RRT rrt(start, end, stepSize, goalRadius, maxIterations, minX, maxX, minY, maxY);
    rrt.generate();

    return 0;
}
