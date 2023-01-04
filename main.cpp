#include <iostream>
#include "net_config.h"
#include "layout.h"

int main(){
    std::vector<std::pair<int, Net_config>> net_configs;
    int test_num = 10;
    for(int i = 0; i < test_num; ++i){
        net_configs.clear();
        Layout L(50, 50, 2, i);
        L.generateObstacles({4,4}, {{3, 10},{3,10}});
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