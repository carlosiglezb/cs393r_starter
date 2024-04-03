#include "rrt.h"

int orientation(const Point &p, const Point &q, const Point &r) {
  double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
  if (val == 0) return 0;  // colinear
  return (val > 0) ? 1 : 2; // clock or counterclock wise
}

bool onSegment(const Point &p, const Point &q, const Point &r) {
  return q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
         q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y);
}

static bool checkIntersection(const LineSegment &line1, const LineSegment &line2) {
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


RRT::RRT(double stepSize, double goalRadius, int maxIterations, double minX, double maxX, double minY, double maxY)
        : stepSize(stepSize), goalRadius(goalRadius), maxIterations(maxIterations), minX(minX), maxX(maxX), minY(minY), maxY(maxY) {
  start = Point(minX, minY);
  end = Point(maxX, maxY);
  // clear rrt_path_points
  rrt_path_points.clear();
  parent.push_back(-1); // Start node has no parent
}

RRT::~RRT() = default;

Point RRT::getRandomPoint() {
  double x = minX + (std::rand() % static_cast<int>(maxX - minX + 1));
  double y = minY + (std::rand() % static_cast<int>(maxY - minY + 1));
  return Point(x, y);
}

int RRT::getNearestNodeIndex(const Point& point) {
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

bool RRT::isPathFree(const Point& start, const Point& end) {
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

bool RRT::addNewNode(const Point& randomPoint) {
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

void RRT::saveToFile(const std::string& pathFile, const std::string& treeFile) {
  std::ofstream pathStream(pathFile);
  std::ofstream treeStream(treeFile);

  // Save final path
  rrt_path_points.clear();
  Eigen::Vector2f current_point;
  int currentIndex = nodes.size() - 1;
  while (currentIndex != -1) {
    pathStream << nodes[currentIndex].x << ", " << nodes[currentIndex].y << std::endl;
    currentIndex = parent[currentIndex];
    current_point << nodes[currentIndex].x, nodes[currentIndex].y;
    if (currentIndex != -1) {
      rrt_path_points.push_back(current_point);
    }
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

void RRT::generate(const Point &_start, const Point &_end) {
  start = _start;
  end = _end;
  nodes.push_back(start);
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

void RRT::readObstaclesFromFile(const std::string& filename) {
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

const std::vector<Eigen::Vector2f> RRT::getRRTPathPoints() {
  return rrt_path_points;
}

const Eigen::Vector2f RRT::getRRTPathPoint(const int idx) {
  return rrt_path_points[rrt_path_points.size() - idx];
}

