#ifndef NAV_UTILS_GEOMETRY_TOOLS_HPP
#define NAV_UTILS_GEOMETRY_TOOLS_HPP

#include <cmath>
#include <algorithm>

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


inline int orientation(const Point &p, const Point &q, const Point &r) {
  double val = (q.y - p.y) * (r.x - q.x) - (q.x - p.x) * (r.y - q.y);
  if (val == 0) return 0;
  return (val > 0) ? 1 : 2;
}

inline bool onSegment(const Point &p, const Point &q, const Point &r) {
  return q.x <= std::max(p.x, r.x) && q.x >= std::min(p.x, r.x) &&
         q.y <= std::max(p.y, r.y) && q.y >= std::min(p.y, r.y);
}

inline bool checkIntersection(const LineSegment &line1, const LineSegment &line2) {
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

#endif //NAV_UTILS_GEOMETRY_TOOLS_HPP
