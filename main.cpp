#include <iostream>
#include "net_config.h"
#include "layout.h"

int main(){
    std::vector<std::pair<int, Net_config>> net_configs;
    int test_num = 10;
    for(int i = 0; i < test_num; ++i){
        net_configs.clear();
        Layout L(50, 50, 2, i);
        L.GenerateObstacle({4,4}, {{3, 10},{3,10}});
        L.AutoConfig(net_configs);
        L.GenerateNets(net_configs);
#ifdef DEBUG
        L.CheckLegal();
#endif
        std::string file_name = "test" + std::to_string(i) + ".txt";
        L.SaveResult(file_name);
        std::cout << std::endl;
    }
    return 0;
}