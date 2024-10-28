#include "layout.h"

#define MAX_LAYER 2  // this limits searchEngine to only route on layer 0 & 1

Layout::Layout(int _width, int _height, int _layers, int idx) : layout_idx(idx), width(_width), height(_height), layers(_layers), length(_width * _height * _layers), r_gen(idx + time(0)){
   // assert(layers == 2);
   grids = new int[length]();
   /** LAYER: only 2 layers */
   h_edges.resize(width);
   for(std::vector<bool> & h_e : h_edges){
      h_e.resize(height - 1, false);
   }
   v_edges.resize(height);
   for(std::vector<bool> & v_e : v_edges){
      v_e.resize(width - 1, false);
   }
   visited = new bool[length]();
}

Layout::~Layout(){
   for(Net * net : nets){
      delete net;
   }
   if(grids != nullptr){
      delete [] grids;
   }
   if(visited != nullptr){
      delete [] visited;
   }
}

void Layout::autoConfig(std::vector<std::pair<int, Net_config>> & net_configs, int net_num, int pin_num){
   const int size = std::max(width, height);
   const size_t min_wl = 5, max_wl = size * 0.75, wl_limit = size * 1.5;
   const int reroute_num = size * 0.15 * net_num * pin_num;
   const float momentum = 0.85;
   Net_config net_config(min_wl, max_wl, wl_limit, pin_num, reroute_num, momentum);
   net_configs.push_back({net_num, net_config});
}

void Layout::generateObstacles(const std::vector<int> & obs_num, const std::vector<std::pair<int,int>> & obs_size_range){
   assert((int)obs_num.size() <= layers);
   assert(obs_num.size() == obs_size_range.size());
   for(int i = 0; i < (int)obs_num.size(); ++i){
      for (int j = 0, counter = 0; j < obs_num[i] && counter < 10;) {
         int obs_w = 1, obs_h = 1;
         if(i % 2){//vertical layer
            obs_h = randInt(r_gen, obs_size_range[i].first, obs_size_range[i].second);
         }else{
            obs_w = randInt(r_gen, obs_size_range[i].first, obs_size_range[i].second);
         }
         int x = randInt(r_gen, 0, width - obs_w - 1);
         int y = randInt(r_gen, 0, height - obs_h - 1);
         Point obs_p1(x, y, i);
         Point obs_p2(x + obs_w, y + obs_h, i);
         if(!addObstacle(obs_p1, obs_p2)){
            counter++;
         }else{
            j++;
         }
      }
   }
}

int Layout::generateNets(const std::vector<std::pair<int, Net_config>> & net_configs){
   int total_nets = 0;
   for(const std::pair<int, Net_config> & net_config : net_configs){
      int counter = 0;
      for(int i = 0; i < net_config.first; ++i){
         if(generateNet(net_config.second)){
            counter++;
            total_nets++;
         }
      }
      std::cout << "Created " << net_config.second.pin_num << " pins net: " << counter << std::endl;
   }
   return total_nets;
}

bool Layout::addObstacle(Point & p1, Point & p2){
   M_Assert(p1.x <= p2.x && p1.y <= p2.y && p1.z <= p2.z, "invalid obstacle");
   //check if position is legal
   for(int x = p1.x; x < p2.x; x++){
      for(int y = p1.y; y < p2.y; y++){
         for(int z = p1.z; z <= p2.z; z++){
            if(getGrid(x, y, z) != 0){
               return false;
            }
         }
      }
   }
   //set obstacle in the grid map
   for(int x = p1.x; x < p2.x; x++){
      for(int y = p1.y; y < p2.y; y++){
         for(int z = p1.z; z <= p2.z; z++){
            setGrid(x, y, z, -1);
         }
      }
   }
   obstacles.push_back({p1, p2});
   return true;
}

void Layout::archiveAndReset(){
   for(Net * n : nets){
      n->reset();
   }
   h_edges.clear();
   v_edges.clear();
   delete [] grids;
   grids = nullptr;
   delete [] visited;
   visited = nullptr;
}

bool Layout::generateNet(const Net_config & config){
   assert(config.pin_num >= 2);//some function doesn't support more than 2 layers
   std::vector<int>candidates_idx;
   
   for(int i = 0; i < length; ++i){
      if(grids[i] == 0 && i % layers == 0){//empty grid at the bottom layer
         candidates_idx.push_back(i);
      }
   }
   
   std::vector<Point>total_path;
   std::vector<Point>n_vias;
   std::vector<Point>n_pins;
   std::shuffle(candidates_idx.begin(), candidates_idx.end(), r_gen);
   for(int i = 0; i < std::min(config.reroute_num,(int)candidates_idx.size()); ++i){
      int idx = candidates_idx[i];
      int x = idx / (height * layers);
      int y = (idx % (height * layers)) / layers;
      int z = idx % layers;  
      //beg_candidates.push_back(Point(x, y, z));
      Point beg(x, y, z);
      Point result = searchEngine(beg, randIntNorm(r_gen, config.min_wl, config.max_wl), config.wl_limit, config.momentum1, total_path, n_vias);
      if(result.x != -1){
         // neighbor pins might make the net "redundant" during training
         // triggers assertion fail: "net_queue->size()"
         if (std::abs(beg.x - result.x) == 1 && beg.y == result.y) continue;
         M_Assert(result.z == 0, "result.z == 0");  // under layer = 2
         n_pins.push_back(beg);
         n_pins.push_back(result);
         setGrid(beg.x, beg.y, beg.z, 2);
         setGrid(result.x, result.y, result.z, 2);
         break;
      }
   }
   if(n_pins.empty()){
      recoverGridAndEdge(total_path);
      return false;
   }

   //route the rest pins
   for(int i = 2; i < config.pin_num; ++i){
      std::vector<Point>candidates_beg;
      for(Point & p_in_path : total_path){
         int status = getGrid(p_in_path.x, p_in_path.y, p_in_path.z);
         if(status == 1 || status == 2){//wire or pin
            candidates_beg.push_back(p_in_path);
            M_Assert(p_in_path.z <= 1, "p_in_path.z <= 1");  // under layer = 2
         }
      }
      std::shuffle(candidates_beg.begin(), candidates_beg.end(), r_gen);
      for(int j = 0; j < std::min(config.reroute_num, (int)candidates_beg.size()); ++j){
         int x = candidates_beg[j].x;
         int y = candidates_beg[j].y;
         int z = candidates_beg[j].z;
         Point result = searchEngine(Point(x, y, z), randIntNorm(r_gen, config.min_wl, config.max_wl), config.wl_limit, config.momentum2, total_path, n_vias);
         if(result.x != -1){
            // neighbor pins might make the net "redundant" during training
            // triggers assertion fail: "net_queue->size()"
            if ((result.x - 1 >= 0) && (getGrid(result.x - 1, result.y, result.z) == 2)) continue;
            if ((result.x + 1 < width) && (getGrid(result.x + 1, result.y, result.z) == 2)) continue;
            M_Assert(result.z == 0, "result.z == 0");  // under layer = 2
            n_pins.push_back(result);
            setGrid(result.x, result.y, result.z, 2);
            break;
         }
      }
      if((int)n_pins.size() != i + 1){
         recoverGridAndEdge(total_path);
         return false;
      }
   }
   Net * net = new Net(nets.size(), n_pins);
   M_Assert((int)net->pins.size() == config.pin_num, "pin number error");

   nets.push_back(net);
   path2Wire(net, n_vias);
   return true;
}

Point Layout::searchEngine(const Point & beg, size_t wl_lower_bound, size_t wl_upper_bound, float momentum, std::vector<Point> & total_path, std::vector<Point> & n_vias){
   std::vector<Point>path;
   std::vector<std::pair<Point, Point>> candidates; //next point, previous point
   candidates.push_back({beg, beg});
   resetVisited();
   while(candidates.size()){
      Point curr_p = candidates.back().first;
      candidates.pop_back();
      size_t pre_size = candidates.size();
      path.push_back(curr_p);
      int x = curr_p.x;
      int y = curr_p.y;
      int z = curr_p.z;
      setVisited(x, y, z);
      setGrid(x, y, z, 1);

      if(path.size() >= wl_lower_bound){
         bool flag = true;
         size_t path_size = path.size();
         M_Assert(z <= 1, "z <= 1");  // under layer = 2
         for(int z_i = z - 1; z_i >=0; --z_i){
            if(getGrid(x, y, z_i) != 0){
               flag = false;
               break;
            }
            setGrid(x, y, z_i, 1);
            path.push_back(Point(x, y, z_i));
         }
         if(flag){
            total_path.push_back(path.front());
            for(size_t i = 1; i < path.size(); ++i){
               Point & p_in_path = path[i];
               M_Assert((p_in_path - path[i-1]).manh()==1, "path error");
               if(p_in_path.x != path[i-1].x){//h_wire
                  v_edges[p_in_path.y][std::min(p_in_path.x, path[i-1].x)] = true;
               }else if(p_in_path.y != path[i-1].y){//v_wire
                  h_edges[p_in_path.x][std::min(p_in_path.y, path[i-1].y)] = true;
               }else{
                  n_vias.push_back(Point(p_in_path.x, p_in_path.y, std::min(p_in_path.z, path[i-1].z)));
               }
               total_path.push_back(p_in_path);
            }
            return Point(x, y, 0);
         }else{//recover grid mark
            while(path.size() != path_size){
               Point & tail = path.back();
               setGrid(tail.x, tail.y, tail.z, 0);
               path.pop_back();
            }
         }
      }
      if(path.size() > wl_upper_bound){
         return Point(-1,-1,-1);
      }

      std::vector<Point> tmp1;
      std::vector<Point> tmp2;
      if(z % 2){
         if(y - 1 >= 0 && getGrid(x, y - 1, z) == 0 && !getVisited(x, y - 1, z)){
            tmp1.push_back(Point(x, y - 1, z));
         }
         if(y + 1 < height && getGrid(x, y + 1, z) == 0 && !getVisited(x, y + 1, z)){
            tmp1.push_back(Point(x, y + 1, z));
         }
         if(z + 1 < MAX_LAYER && getGrid(x, y, z + 1) == 0 && !getVisited(x, y, z + 1)){
            tmp2.push_back(Point(x, y, z + 1));
         }
         if(z - 1 >= 0 && getGrid(x, y, z - 1) == 0 && !getVisited(x, y, z - 1)){
            tmp2.push_back(Point(x, y, z - 1));
         }
      }else{
         if(x - 1 >= 0 && getGrid(x - 1, y, z) == 0 && !getVisited(x - 1, y, z)){
            tmp1.push_back(Point(x - 1, y, z));
         }
         if(x + 1 < width && getGrid(x + 1, y, z) == 0 && !getVisited(x + 1, y, z)){
            tmp1.push_back(Point(x + 1, y, z));
         }
         if(z + 1 < MAX_LAYER && getGrid(x, y, z + 1) == 0 && !getVisited(x, y, z + 1)){
            tmp2.push_back(Point(x, y, z + 1));
         }
         if(z - 1 >= 0 && getGrid(x, y, z - 1) == 0 && !getVisited(x, y, z - 1)){
            tmp2.push_back(Point(x, y, z - 1));
         }
      }
      if(tmp1.size() == 2){
         int dist1 = (tmp1[0] - beg).manh();
         int dist2 = (tmp1[1] - beg).manh();
         if(dist1 > dist2 || (dist1 == dist2 && randFloat(r_gen) > 0.5)){
            std::swap(tmp1[0], tmp1[1]);
         }
      }
      if(tmp2.size() == 2){
         int dist1 = (tmp2[0] - beg).manh();
         int dist2 = (tmp2[1] - beg).manh();
         if(dist1 > dist2 || (dist1 == dist2 && randFloat(r_gen) > 0.5)){
            std::swap(tmp2[0], tmp2[1]);
         }
      }
      if(randFloat(r_gen) > momentum){//change direction
         for(Point & p : tmp1){
            candidates.push_back({p, curr_p});
         }
         for(Point & p : tmp2){
            candidates.push_back({p, curr_p});
         }
      }else{
         for(Point & p : tmp2){
            candidates.push_back({p, curr_p});
         }
         for(Point & p : tmp1){
            candidates.push_back({p, curr_p});
         }
      }
      if(candidates.size() == pre_size && candidates.size()){//no way to go
         Point head = path.back();
         while(head != candidates.back().second){
            setGrid(head.x, head.y, head.z, 0);
            path.pop_back();
            M_Assert(path.size(), "path size must > 0");
            head = path.back();
         }
      }
   }
   return Point(-1,-1,-1);
}

void Layout::recoverGridAndEdge(const std::vector<Point> & total_path){
   for(const Point & p : total_path){
      setGrid(p.x, p.y, p.z, 0);
   }
   for(std::vector<bool> & h_e : h_edges){
      for(size_t i = 0; i < h_e.size(); ++i){
         h_e[i] = false;
      }
   }
   for(std::vector<bool> & v_e : v_edges){
      for(size_t i = 0; i < v_e.size(); ++i){
         v_e[i] = false;
      }
   }
}


void Layout::path2Wire(Net *n, std::vector<Point>& n_vias){
   n->vias.swap(n_vias);
   n->wl = n->vias.size();
   for(int i = 0; i < width; ++i){
      int beg_idx = -1;
      for(int j = 0; j < height - 1; ++j){
         if(!h_edges[i][j]){
            if(j - beg_idx >= 2){
               n->v_segments.push_back({i, beg_idx + 1, 1, i + 1, j + 1, 1});
               n->wl += j - beg_idx -1;
            }
            beg_idx = j;
         }
         h_edges[i][j] = false;
      }
      if(height - 1 - beg_idx >= 2){
         n->v_segments.push_back({i, beg_idx + 1, 1, i + 1, height, 1});
         n->wl += height - 1 - beg_idx -1;
      }
   }

   for(int i = 0; i < height; ++i){
      int beg_idx = -1;
      for(int j = 0; j < width - 1; ++j){
         if(!v_edges[i][j]){
            if(j - beg_idx >= 2){
               n->h_segments.push_back({beg_idx + 1, i, 0, j + 1, i + 1, 0});
               n->wl += j - beg_idx -1;
            }
            beg_idx = j;
         }
         v_edges[i][j] = false;
      }
      if(width - 1 - beg_idx >= 2){
         n->h_segments.push_back({beg_idx + 1, i, 0, width, i + 1, 0});
         n->wl += width - 1 - beg_idx -1;
      }
   }
}

void Layout::saveResult(const std::string & filename){
   std::ofstream fout;
	fout.open(filename, std::ofstream::out);

	if (!fout.is_open())
	{	
		std::cerr << "Cannot save the result." << std::endl;
		std::cerr << "Please check." << std::endl;
		exit(1);
	}

	fout << "Width 0 " << width << std::endl;
	fout << "Height 0 " << height << std::endl;

   int total_wl = 0, total_via = 0;
   for(Net * n : nets){
      total_wl += n->wl;
      total_via += n->vias.size();
   }
	fout << "total_WL " << total_wl << std::endl;
   fout << "total_via " << total_via << std::endl;;
	fout << "Layer " << layers << std::endl;
   for (int i = 0; i < layers; i++) {
      fout << "track" << i << " 0 1 " << (i % 2) << std::endl;
   }
   fout << "Obstacle_num " << obstacles.size() << std::endl;
   for(std::pair<Point, Point> & p : obstacles){
		fout << p.first.x << " " << p.first.y << " " << p.first.z << " " << p.second.x << " " << p.second.y << " " << p.second.z << std::endl;
	}
   fout << "Net_num " << nets.size() << std::endl;
   for(Net * n : nets){
      fout << "Net_id " << n->net_id << std::endl;
      fout << "pin_num " << n->pins.size() << std::endl;
      int pid = 0;
      for(Point & p : n->pins){
         fout << "pin_id " << pid++ << std::endl;
         fout << "ap_num 1" << std::endl;
		   fout << p.x << " " << p.y << " " << p.z << std::endl;
	   }
      fout << "Via_num " << n->vias.size() << std::endl;
      for(Point & p : n->vias){
         fout << p.x << " " << p.y << std::endl;
      }
      fout << "H_segment_num " << n->h_segments.size() << std::endl;
      for(std::vector<int> & seg : n->h_segments){
         fout << seg[0] << " " << seg[1] << " " << seg[2] << " " << seg[3] << " " << seg[4] << " " << seg[5] << std::endl;
      }
      fout << "V_segment_num " << n->v_segments.size() << std::endl;
      for(std::vector<int> & seg : n->v_segments){
         fout << seg[0] << " " << seg[1] << " " << seg[2] << " " << seg[3] << " " << seg[4] << " " << seg[5] << std::endl;
      }
   }

	fout.close();
}

void Layout::checkLegal(){
   bool test_grid[width][height][layers];
   for(int x = 0; x < width; ++x){
      for(int y = 0; y < height; ++y){
         for(int z = 0; z < layers; ++z){
            test_grid[x][y][z] = false;
         }
      }
   }
   for(Net * n : nets){
      for(std::vector<int> & seg : n->h_segments){
         int y = seg[1];
         int z = seg[2];
         for(int x = seg[0]; x < seg[3]; ++x){
            if(test_grid[x][y][z]){
               std::cerr << "H Error: " << x << " " << y << " " << z << std::endl;
            }
            test_grid[x][y][z] = true;
         }
      }
      for(std::vector<int> & seg : n->v_segments){
         int x = seg[0];
         int z = seg[2];
         for(int y = seg[1]; y< seg[4]; ++y){
            if(test_grid[x][y][z]){
               std::cerr << "V Error: " << x << " " << y << " " << z << std::endl;
            }
            test_grid[x][y][z] = true;
         }
      }
      for(Point & p : n->pins){
         test_grid[p.x][p.y][p.z] = true;
      }
   }
}