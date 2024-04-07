#include "rrt_star.h"

int main() {
  // Point start(-35.50, 19.00), end(40.25, 26.27);
  Point start(40.25, 26.27), end(-35.50, 19.00);
  // Point start(-38.172, 12.61), end(-9.5, 13.7);

  double stepSize = 1;
  double goalRadius = 1.0;
  double minDistanceToObstacle = 0.2;
  int maxIterations = 100000000;

  double minX = -45, maxX = 45, minY = -30, maxY = 30;

  RRTStar rrt(stepSize, goalRadius, maxIterations, minX, maxX, minY, maxY, minDistanceToObstacle);
  rrt.readObstaclesFromFile("../../../../../amrl_maps/GDC3/GDC3.vectormap.txt");
  rrt.generate(start, end);

  return 0;
}
