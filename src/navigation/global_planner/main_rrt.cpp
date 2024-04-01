#include "rrt.h"

int main() {
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
    rrt.readObstaclesFromFile("../../../../../amrl_maps/GDC3/GDC3.vectormap.txt"); // You may need to change the path to the file
    rrt.generate();

    return 0;
}
