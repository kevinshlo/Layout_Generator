#ifndef _NET_H_
#define _NET_H_
#include <vector>
#include "point.h"

class Net{
public:
   Net(int id, std::vector<Point> pins_);
   ~Net();
   const size_t net_id;
   int ori_wl;
   std::vector<Point> pins;
   std::vector<Point> vias;//z coordinate is the bottom layer of the via
   std::vector<std::vector<int>> h_segments;
   std::vector<std::vector<int>> v_segments;
};

#endif