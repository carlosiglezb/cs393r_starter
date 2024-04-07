#include "rrt_star.h"

int main() {
  // Point start(-36.41, 15.88), end(13.8, 8.9);
  Point start(-33.64, 20.44), end(-1.8, 19.14);


  double stepSize = 0.5;
  double goalRadius = 1.0;
  float minDistanceToObstacle = 0.2;
  int maxIterations = 100000000;

  double minX = -45, maxX = 45, minY = -30, maxY = 30;

  RRTStar rrt(stepSize, goalRadius, maxIterations, minX, maxX, minY, maxY, minDistanceToObstacle);
  rrt.readObstaclesFromFile("./GDC1.vectormap.txt");
  rrt.generate(start, end);

  // create obstacle from waypoint
  Eigen::Vector2f waypoint(-25.323, 16.167);

  // test re-planing
  rrt.reset();
  start = Point(-25.89, 16.156);
  rrt.labelAsObstacle(waypoint);
  rrt.generate(start, end);

  return 0;
}
