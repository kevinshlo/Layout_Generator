#ifndef _LAYOUT_H
#define _LAYOUT_H

#include <iostream>
#include <random>
#include <ctime>
#include <vector>
#include <queue>
#include <set>
#include <algorithm>
#include <cstring>
#include <fstream>
#include <stack>
#include "point.h"
#include "net.h"
#include "assert.h"
#include "net_config.h"
#include "debugger.h"
inline int randInt(int min, int max){
   return rand() % (max - min + 1) + min;
}

inline int randInt(std::mt19937 & generator, int min, int max){
   std::uniform_int_distribution<int> distribution(min, max);
   return distribution(generator);
}

inline float randFloat(std::mt19937 & generator, float min = 0.0, float max = 1.0){
   std::uniform_real_distribution<float> distribution(min, max);
   return distribution(generator);
}

class Layout{
public:
   Layout(int _width, int _height, int _layers, int idx);
   ~Layout();
   inline void SetGrid(int x, int y, int z, int value){
      M_Assert(x >= 0 && x < width && y >= 0 && y < height && z >= 0 && z < layers, "out of range");
      grids[x * height * layers + y * layers + z] = value;
   }
   inline int GetGrid(int x, int y, int z) const{
      M_Assert(x >= 0 && x < width && y >= 0 && y < height && z >= 0 && z < layers, "out of range");
      return grids[x * height * layers + y * layers + z];
   }
   inline void SetVisited(int x, int y, int z){
      M_Assert(x >= 0 && x < width && y >= 0 && y < height && z >= 0 && z < layers, "out of range");
      visited[x * height * layers + y * layers + z] = true;
   }
   inline bool GetVisited(int x, int y, int z) const{
      M_Assert(x >= 0 && x < width && y >= 0 && y < height && z >= 0 && z < layers, "out of range");
      return visited[x * height * layers + y * layers + z];
   }
   inline void ResetVisited(){
      memset(visited, 0, sizeof(bool) * length);
   }
   void Initialize(const std::vector<int> & obs_num, const std::vector<std::pair<int,int>> & obs_size_range, const std::vector<std::pair<int, Net_config>> & net_configs);
   bool AddObstacle(Point & p1, Point & p2);
   bool GenerateNet(const Net_config & config);
   bool SearchEngine(Net *net, const Point & beg, size_t wl_lower_bound, size_t wl_upper_bound, float momentum, std::vector<Point> & total_path);
   void Path2Wire(Net * n, std::vector<Point> & total_path);
   void SaveResult(const std::string & filename);
   void CheckLegal();
   //std::vector<Point> pins;
   std::vector<Net *> nets;
   //std::vector<Point> vias;//z coordinate is the bottom layer of the via
   std::vector<std::pair<Point, Point>>obstacles;
   //std::vector<std::vector<int>> h_segments;
   //std::vector<std::vector<int>> v_segments;
private:
   const int width;
   const int height;
   const int layers;
   const int length;
   const int layout_idx;
   std::mt19937 r_gen;
   int * grids; //-1: obstacle, 0: empty, 1: net 2: pin
   bool * visited;
};

#endif