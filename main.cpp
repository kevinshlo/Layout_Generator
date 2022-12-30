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
        L.autoConfig(net_configs);
        L.generateNets(net_configs);
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