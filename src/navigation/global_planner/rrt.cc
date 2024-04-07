#include "rrt.h"

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

