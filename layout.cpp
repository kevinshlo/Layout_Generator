#include "layout.h"

Layout::Layout(int _width, int _height, int _layers, int idx) : width(_width), height(_height), layers(_layers), length(_width * _height * _layers), layout_idx(idx), r_gen(idx){
   grids = new int[length]();
   visited = new bool[length]();
}

Layout::~Layout(){
   for(Net * net : nets){
      delete net;
   }
   delete [] grids;
   delete [] visited;
}

void Layout::Initialize(const std::vector<int> & obs_num, const std::vector<std::pair<int,int>> & obs_size_range, const std::vector<std::pair<int, Net_config>> & net_configs){
   assert((int)obs_num.size() == layers);
   assert(obs_num.size() == obs_size_range.size());
   for(int i = 0; i < layers; ++i){
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
         if(!AddObstacle(obs_p1, obs_p2)){
            counter++;
         }else{
            if((int)obstacles.size() - pre_obs_num == obs_num[i]){
               break;
            }
         }
      }
   }

   for(const std::pair<int, Net_config> & net_config : net_configs){
      for(int i = 0; i < net_config.first; ++i){
         GenerateNet(net_config.second);
      }
   }
}


bool Layout::AddObstacle(Point & p1, Point & p2){
   M_Assert(p1.x <= p2.x && p1.y <= p2.y && p1.z <= p2.z, "invalid obstacle");
   //check if position is legal
   for(int x = p1.x; x < p2.x; x++){
      for(int y = p1.y; y < p2.y; y++){
         for(int z = p1.z; z <= p2.z; z++){
            if(GetGrid(x, y, z) != 0){
               return false;
            }
         }
      }
   }
   //set obstacle in the grid map
   for(int x = p1.x; x < p2.x; x++){
      for(int y = p1.y; y < p2.y; y++){
         for(int z = p1.z; z <= p2.z; z++){
            SetGrid(x, y, z, -1);
         }
      }
   }
   obstacles.push_back({p1, p2});
   return true;
}

bool Layout::GenerateNet(const Net_config & config){
   assert(config.pin_num >= 2);
   //srand(seed);
   //std::mt19937 generator(seed);
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
   std::random_shuffle(candidates_idx.begin(), candidates_idx.end());
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
      success = SearchEngine(net, beg, randInt(r_gen, config.min_wl, config.max_wl), config.wl_limit, config.momentum1, total_path);
      if(success){
         net->pins.push_back(beg);
         SetGrid(beg.x, beg.y, beg.z, 2);
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
            if(GetGrid(x, y, z) == 1){//wire grid at the bottom layer
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
         success = SearchEngine(net, beg, randInt(r_gen, config.min_wl, config.max_wl), config.wl_limit, config.momentum2, total_path);
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
   Path2Wire(net, total_path);
   return true;
}

bool Layout::SearchEngine(Net *net, const Point & beg, size_t wl_lower_bound, size_t wl_upper_bound, float momentum, std::vector<Point> & total_path){
   std::vector<Point>path;
   std::vector<std::pair<Point, Point>> candidates; //next point, previous point
   //std::vector<Point> record;
   candidates.push_back({beg, beg});
   //record.push_back(beg);
   ResetVisited();
   while(candidates.size()){
      Point curr_p = candidates.back().first;
      candidates.pop_back();
      //record.pop_back();
      size_t pre_size = candidates.size();
      path.push_back(curr_p);
      int x = curr_p.x;
      int y = curr_p.y;
      int z = curr_p.z;
      SetVisited(x, y, z);
      SetGrid(x, y, z, 1);

      if(path.size() >= wl_lower_bound){
         bool flag = true;
         size_t path_size = path.size();
         for(int z_i = z - 1; z_i >=0; --z_i){
            if(GetGrid(x, y, z_i) != 0){
               flag = false;
               break;
            }
            SetGrid(x, y, z_i, 2);
            path.push_back(Point(x, y, z_i));
         }
         if(flag){
            net->pins.push_back(Point(x, y, 0));
            total_path.push_back(path.front());
            for(size_t i = 1; i < path.size(); ++i){
               Point & p_in_path = path[i];
               if(p_in_path.x == path[i-1].x && p_in_path.y == path[i-1].y){
                  net->vias.push_back(Point(p_in_path.x, p_in_path.y, std::min(p_in_path.z, path[i-1].z)));
               }
               M_Assert((p_in_path - path[i-1]).manh()==1, "path error");
               total_path.push_back(p_in_path);
            }
            //std::cout << "end pin: " << x << "," << y << "," << 0 << std::endl;
            return true;
         }else{//recover grid mark
            while(path.size() != path_size){
               Point & tail = path.back();
               SetGrid(tail.x, tail.y, tail.z, 0);
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
         if(y - 1 >= 0 && GetGrid(x, y - 1, z) == 0 && !GetVisited(x, y - 1, z)){
            tmp1.push_back(Point(x, y - 1, z));
         }
         if(y + 1 < height && GetGrid(x, y + 1, z) == 0 && !GetVisited(x, y + 1, z)){
            tmp1.push_back(Point(x, y + 1, z));
         }
         if(z + 1 < layers && GetGrid(x, y, z + 1) == 0 && !GetVisited(x, y, z + 1)){
            tmp2.push_back(Point(x, y, z + 1));
         }
         if(z - 1 >= 0 && GetGrid(x, y, z - 1) == 0 && !GetVisited(x, y, z - 1)){
            tmp2.push_back(Point(x, y, z - 1));
         }
      }else{
         if(x - 1 >= 0 && GetGrid(x - 1, y, z) == 0 && !GetVisited(x - 1, y, z)){
            tmp1.push_back(Point(x - 1, y, z));
         }
         if(x + 1 < width && GetGrid(x + 1, y, z) == 0 && !GetVisited(x + 1, y, z)){
            tmp1.push_back(Point(x + 1, y, z));
         }
         if(z + 1 < layers && GetGrid(x, y, z + 1) == 0 && !GetVisited(x, y, z + 1)){
            tmp2.push_back(Point(x, y, z + 1));
         }
         if(z - 1 >= 0 && GetGrid(x, y, z - 1) == 0 && !GetVisited(x, y, z - 1)){
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
            //record.push_back(curr_p);
         }
         for(Point & p : tmp2){
            candidates.push_back({p, curr_p});
            //record.push_back(curr_p);
         }
      }else{
         for(Point & p : tmp2){
            candidates.push_back({p, curr_p});
            //record.push_back(curr_p);
         }
         for(Point & p : tmp1){
            candidates.push_back({p, curr_p});
            //record.push_back(curr_p);
         }
      }
      if(candidates.size() == pre_size && candidates.size()){//no way to go
         Point head = path.back();
         while(head != candidates.back().second){
            SetGrid(head.x, head.y, head.z, 0);
            path.pop_back();
            M_Assert(path.size(), "path size must > 0");
            head = path.back();
         }
      }
   }
   return false;
}

void Layout::Path2Wire(Net * n, std::vector<Point> & path){
   M_Assert(path.size() >= 2, "path size must >= 2");
   for(Point & p : path){
      SetGrid(p.x, p.y, p.z, 3);
   }
   for(int z = 0; z < layers; z += 2){
      for(int y = 0; y < height; ++y){
         int beg_idx = 0;
         for(int x = 0; x < width; ++x){
            if(GetGrid(x, y, z) != 3){
               if(x - beg_idx > 1){
                  n->h_segments.push_back({beg_idx, y, z, x, y + 1, z});
                  n->ori_wl += x - beg_idx -1;
               }
               beg_idx = x + 1;
            }
         }
         if(width - beg_idx > 1){
            n->h_segments.push_back({beg_idx, y, z, width, y + 1, z});
            n->ori_wl += width - beg_idx -1;
         }
      }
   }

   for(int z = 1; z < layers; z += 2){
      for(int x = 0; x < width; ++x){
         int beg_idx = 0;
         for(int y = 0; y < height; ++y){
            if(GetGrid(x, y, z) != 3){
               if(y - beg_idx > 1){
                  n->v_segments.push_back({x, beg_idx, z, x + 1, y, z});
                  n->ori_wl += y - beg_idx -1;
               }
               beg_idx = y + 1;
            }
         }
         if(height - beg_idx > 1){
            n->v_segments.push_back({x, beg_idx, z, x + 1, height, z});
            n->ori_wl += height - beg_idx -1;
         }
      }
   }
   for(Point & p : path){
      SetGrid(p.x, p.y, p.z, 1);
   }
   for(Point & p : n->pins){
      SetGrid(p.x, p.y, p.z, 2);
   }
}


void Layout::SaveResult(const std::string & filename){
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
      total_wl += n->ori_wl;
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

void Layout::CheckLegal(){
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