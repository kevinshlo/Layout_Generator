#include "layout.h"

Layout::Layout(int _width, int _height, int _layers, int idx) : width(_width), height(_height), layers(_layers), length(_width * _height * _layers), layout_idx(idx), r_gen(idx){
   assert(layers == 2);
   grids = new int[length]();
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

void Layout::autoConfig(std::vector<std::pair<int, Net_config>> & net_configs){
   size_t wl_limit = std::max(width, height) * 1.5;
   Net_config net_2_pins(5,  25, wl_limit, 2, 20, 0.85);
   Net_config net_3_pins(7,  20, wl_limit, 3, 15, 0.85);
   Net_config net_4_pins(7,  20, wl_limit, 4, 10, 0.85);
   Net_config net_5_pins(5,  20, wl_limit, 5, 10, 0.85);
   std::normal_distribution<float> distribution_2(30.0, 2.0);
   std::normal_distribution<float> distribution_3(10.0, 1.5);
   std::normal_distribution<float> distribution_4(5.0, 1.0);
   std::normal_distribution<float> distribution_5(5.0, 1.0);
   net_configs.push_back({(int) std::round(distribution_5(r_gen)), net_5_pins});
   net_configs.push_back({(int) std::round(distribution_4(r_gen)), net_4_pins});
   net_configs.push_back({(int) std::round(distribution_3(r_gen)), net_3_pins});
   net_configs.push_back({(int) std::round(distribution_2(r_gen)), net_2_pins});
}

void Layout::generateObstacles(const std::vector<int> & obs_num, const std::vector<std::pair<int,int>> & obs_size_range){
   assert((int)obs_num.size() <= layers);
   assert(obs_num.size() == obs_size_range.size());
   for(int i = 0; i < (int)obs_num.size(); ++i){
      int counter = 0;
      int obs_w = 1;
      int obs_h = 1;
      int pre_obs_num = obstacles.size();
      while(counter < 10){
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
            if((int)obstacles.size() - pre_obs_num == obs_num[i]){
               break;
            }
         }
      }
   }
}

void Layout::generateNets(const std::vector<std::pair<int, Net_config>> & net_configs){
   for(const std::pair<int, Net_config> & net_config : net_configs){
      int counter = 0;
      for(int i = 0; i < net_config.first; ++i){
         if(generateNet(net_config.second)){
            counter++;
         }
      }
      std::cout << "Created " << net_config.second.pin_num << " pins net: " << counter << std::endl;
   }
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
   std::vector<Point>beg_candidates;
   std::vector<int>candidates_idx;
   
   for(int i = 0; i < length; ++i){
      if(grids[i] == 0 && i % layers == 0){//empty grid at the bottom layer
         candidates_idx.push_back(i);
      }
   }
   if(candidates_idx.empty()){
      return false;
   }
   std::shuffle(candidates_idx.begin(), candidates_idx.end(), r_gen);
   for(int i = 0; i < std::min(config.reroute_num,(int)candidates_idx.size()); ++i){
      int idx = candidates_idx[i];
      int x = idx / (height * layers);
      int y = (idx % (height * layers)) / layers;
      int z = idx % layers;  
      beg_candidates.push_back(Point(x, y, z));
   }
   bool success = false;
   std::vector<Point>total_path;
   Net * net = new Net(nets.size(), {});
   for(Point beg : beg_candidates){
      success = searchEngine(net, beg, randInt(r_gen, config.min_wl, config.max_wl), config.wl_limit, config.momentum1, total_path);
      if(success){
         net->pins.push_back(beg);
         setGrid(beg.x, beg.y, beg.z, 2);
         break;
      }
   }
   if(!success){
      delete net;
      return false;
   }
   //route the rest pins
   for(int i = 2; i < config.pin_num; ++i){
      success = false;
      beg_candidates.clear();
      for(int j = 0; j < config.reroute_num; ++j){
         int beg_idx = randInt(r_gen, 0, total_path.size() - 1);
         for(int k = beg_idx; k < beg_idx + (int)total_path.size(); k++){
            int idx = k % total_path.size();
            int x = total_path[idx].x;
            int y = total_path[idx].y;
            int z = total_path[idx].z;
            if(getGrid(x, y, z) == 1){//wire grid at the bottom layer
               beg_candidates.push_back(Point(x, y, z));
               break;
            }
         }
         if(beg_candidates.empty()){
            delete net;
            return false;
         }
      }
      for(Point beg : beg_candidates){
         success = searchEngine(net, beg, randInt(r_gen, config.min_wl, config.max_wl), config.wl_limit, config.momentum2, total_path);
         if(success){
            break;
         }
      }
      if(!success){
         delete net;
         return false;
      }
   }
   M_Assert((int)net->pins.size() == config.pin_num, "pin number error");
   nets.push_back(net);
   path2Wire(net);
   return true;
}

bool Layout::searchEngine(Net *net, const Point & beg, size_t wl_lower_bound, size_t wl_upper_bound, float momentum, std::vector<Point> & total_path){
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
         for(int z_i = z - 1; z_i >=0; --z_i){
            if(getGrid(x, y, z_i) != 0){
               flag = false;
               break;
            }
            setGrid(x, y, z_i, 2);
            path.push_back(Point(x, y, z_i));
         }
         if(flag){
            net->pins.push_back(Point(x, y, 0));
            total_path.push_back(path.front());
            for(size_t i = 1; i < path.size(); ++i){
               Point & p_in_path = path[i];
               M_Assert((p_in_path - path[i-1]).manh()==1, "path error");
               if(p_in_path.x != path[i-1].x){//h_wire
                  v_edges[p_in_path.y][std::min(p_in_path.x, path[i-1].x)] = true;
               }else if(p_in_path.y != path[i-1].y){//v_wire
                  h_edges[p_in_path.x][std::min(p_in_path.y, path[i-1].y)] = true;
               }else{
                  net->vias.push_back(Point(p_in_path.x, p_in_path.y, std::min(p_in_path.z, path[i-1].z)));
               }
               total_path.push_back(p_in_path);
            }
            return true;
         }else{//recover grid mark
            while(path.size() != path_size){
               Point & tail = path.back();
               setGrid(tail.x, tail.y, tail.z, 0);
               path.pop_back();
            }
         }
      }
      if(path.size() > wl_upper_bound){
         return false;
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
         if(z + 1 < layers && getGrid(x, y, z + 1) == 0 && !getVisited(x, y, z + 1)){
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
         if(z + 1 < layers && getGrid(x, y, z + 1) == 0 && !getVisited(x, y, z + 1)){
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
   return false;
}

void Layout::path2Wire(Net *n){
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

	fout << "Width " << width << std::endl;
	fout << "Height " << height << std::endl;
	fout << "Layer " << layers << std::endl;

   int total_wl = 0;
   for(Net * n : nets){
      total_wl += n->wl;
   }
	fout << "Total_WL " << total_wl << std::endl;
   fout << "Obstacle_num " << obstacles.size() << std::endl;
   for(std::pair<Point, Point> & p : obstacles){
		fout << p.first.x << " " << p.first.y << " " << p.first.z << " " << p.second.x << " " << p.second.y << " " << p.second.z << std::endl;
	}
   fout << "Net_num " << nets.size() << std::endl;
   for(Net * n : nets){
      fout << "Net_id " << n->net_id << std::endl;
      fout << "pin_num " << n->pins.size() << std::endl;
      for(Point & p : n->pins){
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