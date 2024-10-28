#ifndef _LAYOUT_H
#define _LAYOUT_H

#include <iostream>
#include <random>
#include <ctime>
#include <cmath>
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

inline int randInt(std::mt19937 & generator, int min, int max){
   std::uniform_int_distribution<int> distribution(min, max);
   return distribution(generator);
}

inline int randIntNorm(std::mt19937 & generator, int min, int max) {
   std::normal_distribution<float> distribution(0, float(max - min) / 3);
   return std::round(min + std::abs(distribution(generator)));
}

inline float randFloat(std::mt19937 & generator, float min = 0.0, float max = 1.0){
   std::uniform_real_distribution<float> distribution(min, max);
   return distribution(generator);
}

class Layout{
public:
   Layout(int _width, int _height, int _layers, int idx);
   ~Layout();
   
   void autoConfig(std::vector<std::pair<int, Net_config>> & net_configs, int net_num, int pin_num);
   void generateObstacles(const std::vector<int> & obs_num, const std::vector<std::pair<int,int>> & obs_size_range);
   bool addObstacle(Point & p1, Point & p2);
   int generateNets(const std::vector<std::pair<int, Net_config>> & net_configs);
   bool generateNet(const Net_config & config);
   void saveResult(const std::string & filename);
   void checkLegal();

   std::vector<Net *> nets;
   std::vector<std::pair<Point, Point>>obstacles;
   const int layout_idx;
protected:
   void archiveAndReset();//free memory and only keep net pins result, after this function is called, net can't be generated anymore
   const int width;
   const int height;
   const int layers;
   const int length;
private:
   inline void setGrid(int x, int y, int z, int value){
      M_Assert(x >= 0 && x < width && y >= 0 && y < height && z >= 0 && z < layers, "out of range");
      grids[x * height * layers + y * layers + z] = value;
   }
   inline int getGrid(int x, int y, int z) const{
      M_Assert(x >= 0 && x < width && y >= 0 && y < height && z >= 0 && z < layers, "out of range");
      return grids[x * height * layers + y * layers + z];
   }
   inline void setVisited(int x, int y, int z){
      M_Assert(x >= 0 && x < width && y >= 0 && y < height && z >= 0 && z < layers, "out of range");
      visited[x * height * layers + y * layers + z] = true;
   }
   inline bool getVisited(int x, int y, int z) const{
      M_Assert(x >= 0 && x < width && y >= 0 && y < height && z >= 0 && z < layers, "out of range");
      return visited[x * height * layers + y * layers + z];
   }
   inline void resetVisited(){
      memset(visited, 0, sizeof(bool) * length);
   }

   Point searchEngine(const Point & beg, size_t wl_lower_bound, size_t wl_upper_bound, float momentum, std::vector<Point> & total_path, std::vector<Point> & n_vias);
   void path2Wire(Net * n, std::vector<Point>& n_vias);
   void recoverGridAndEdge(const std::vector<Point> & total_path);

   
   std::mt19937 r_gen;
   int * grids; //-1: obstacle, 0: empty, 1: net 2: pin
   std::vector<std::vector<bool>> h_edges;
   std::vector<std::vector<bool>> v_edges;
   bool * visited;
};

#endif