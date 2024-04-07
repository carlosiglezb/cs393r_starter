#include "rrt_star.h"

RRTStar::RRTStar(double stepSize,
                 double goalRadius,
                 int maxIterations,
                 double minX,
                 double maxX,
                 double minY,
                 double maxY,
                 float minDistanceToObstacle)
    : stepSize(stepSize), goalRadius(goalRadius), maxIterations(maxIterations), minX(minX), maxX(maxX), minY(minY), maxY(maxY), minDistanceToObstacle(minDistanceToObstacle) {
    start = Point(minX, minY);
    end = Point(maxX, maxY);
    parent.push_back(-1);
    costs[0] = 0;
    rrt_path_points.clear();
}

RRTStar::~RRTStar() = default;

void RRTStar::reset() {
  nodes.clear();
  parent.clear();
  parent.push_back(-1);
  costs.clear();
  costs[0] = 0;
  rrt_path_points.clear();
}

Point RRTStar::getRandomPoint() {
    double x = minX + (std::rand() % static_cast<int>(maxX - minX + 1));
    double y = minY + (std::rand() % static_cast<int>(maxY - minY + 1));
    return Point(x, y);
}

int RRTStar::getNearestNodeIndex(const Point& point) {
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

std::vector<int> RRTStar::findNearbyNodes(const Point& point, double radius) {
    std::vector<int> nearbyNodes;
    for (size_t i = 0; i < nodes.size(); i++) {
        if (nodes[i].distance(point) <= radius) {
            nearbyNodes.push_back(i);
        }
    }
    return nearbyNodes;
}

double RRTStar::pointToLineSegmentDistance(const Point& p, const LineSegment& line) {
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

bool RRTStar::isPathFree(const Point& newStart, const Point& newEnd, double _minDistanceToObstacle) {
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

    // Check if the closest distances are greater than _minDistanceToObstacle
    if (closestObstacleEndpointDist < _minDistanceToObstacle || closestDistToNewEnd < _minDistanceToObstacle) {
        return false;
    }

    return true;
}

void RRTStar::addNewNode(const Point& randomPoint) {
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

void RRTStar::chooseBestParent(Point& newNode, std::vector<int>& nearbyNodes) {
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

void RRTStar::rewire(std::vector<int>& nearbyNodes, int newNodeIndex) {
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

void RRTStar::populateRRTPathPoints() {
    rrt_path_points.clear();
    int currentIndex = nodes.size() - 1;
    while (currentIndex != -1) {
        Eigen::Vector2f current_point;
        current_point << nodes[currentIndex].x, nodes[currentIndex].y;
        rrt_path_points.push_back(current_point);
        currentIndex = parent[currentIndex];
    }
    std::reverse(rrt_path_points.begin(), rrt_path_points.end());
}

void RRTStar::incrementalSmoothPath() {
    if (rrt_path_points.empty()) return;

    std::vector<Eigen::Vector2f> new_path;
    new_path.push_back(rrt_path_points.front()); // Always include the start point.

    size_t i = 0;
    while (i < rrt_path_points.size() - 1) {
        size_t j = rrt_path_points.size() - 1;
        while (j > i + 1) {
            Eigen::Vector2f start = rrt_path_points[i];
            Eigen::Vector2f end = rrt_path_points[j];
            Point startPoint(start[0], start[1]);
            Point endPoint(end[0], end[1]);
            if (isPathFree(startPoint, endPoint, minDistanceToObstacle)) {
                new_path.push_back(rrt_path_points[j]);
                i = j; // Move to the next segment starting from j.
                break; // Exit the inner loop.
            }
            j--;
        }
        if (j == i + 1) {
            // No direct path is feasible, add the next point and try again from there.
            new_path.push_back(rrt_path_points[i + 1]);
            i++;
        }
    }

    rrt_path_points = new_path; // Update the path with the smoothed version.
}

void RRTStar::interpolateWaypoints() {
    if (rrt_path_points.size() < 2) return; // Ensure there are at least 2 points to interpolate between.

    std::vector<Eigen::Vector2f> new_path;
    for (size_t i = 0; i < rrt_path_points.size() - 1; ++i) {
        Eigen::Vector2f start = rrt_path_points[i];
        Eigen::Vector2f end = rrt_path_points[i + 1];
        new_path.push_back(start); // Always include the segment's start point.

        float segmentLength = (end - start).norm();
        if (segmentLength > stepSize) {
            int numPoints = static_cast<int>(std::ceil(segmentLength / stepSize)) - 1;
            Eigen::Vector2f direction = (end - start).normalized();

            for (int j = 1; j <= numPoints; ++j) {
                Eigen::Vector2f newPoint = start + direction * (stepSize * j);
                new_path.push_back(newPoint);
            }
        }
    }
    new_path.push_back(rrt_path_points.back());// Ensure the last point is included.

    rrt_path_points = new_path; // Update the path with interpolated points.
}

void RRTStar::saveToFile(const std::string& pathFile, const std::string& treeFile) {
    std::ofstream pathStream(pathFile);
    std::ofstream treeStream(treeFile);

    // Save final path
    for (const auto& point : rrt_path_points) {
        pathStream << point[0] << ", " << point[1] << std::endl;
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

void RRTStar::generate(const Point &_start, const Point &_end) {
    start = _start;
    end = _end;
    nodes.push_back(start);
  std::srand(static_cast<unsigned int>(std::time(0)));
    for (int i = 0; i < maxIterations; ++i) {
        Point randomPoint = getRandomPoint();
        addNewNode(randomPoint);
        if (nodes.back().distance(end) < goalRadius) {
            std::cout << "Goal reached." << std::endl;
            break;
        }
    }
    populateRRTPathPoints(); 
    std::cout << "Path points before smoothing: " << rrt_path_points.size() << std::endl;
    incrementalSmoothPath();
    interpolateWaypoints();
    saveToFile("final_path.txt", "tree_structure.txt");
}

void RRTStar::readObstaclesFromFile(const std::string& filename) {
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

const std::vector<Eigen::Vector2f> RRTStar::getRRTPathPoints() {
  return rrt_path_points;
}

const Eigen::Vector2f RRTStar::getRRTPathPoint(const int idx) {
  return rrt_path_points[rrt_path_points.size() - idx];
}
