#include <iostream>
#include "net_config.h"
#include "layout.h"

int main(){
    std::vector<std::pair<int, Net_config>> net_configs;
    int test_num = 10;
    for(int i = 0; i < test_num; ++i){
        net_configs.clear();
        const int layers = 2;
        Layout L(50, 50, layers, i);
        const int obs_num = 4;
        const std::pair<int, int> obs_size_range = {3, 10};
        L.generateObstacles(
            std::vector<int>(layers, obs_num), 
            std::vector<std::pair<int, int>>(layers, obs_size_range)
        );
        //Net_config net_2_pins(10, 35, 60, 2, 20, 0.85);
        L.autoConfig(net_configs);
        L.generateNets(net_configs);
        //L.generateNets({{200, net_2_pins}});
#ifdef DEBUG
        L.checkLegal();
#endif
        std::string file_name = "test" + std::to_string(i) + ".txt";
        L.saveResult(file_name);
        //L.archiveAndReset();
        std::cout << std::endl;
    }
    return 0;
}