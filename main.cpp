#include <iostream>
#include "net_config.h"
#include "layout.h"

int main(){
    Layout L(50, 50, 2, 0);
    Net_config net_2_pins(5, 25, 60, 2, 20, 0.85);
    Net_config net_3_pins(10, 20, 60, 3, 10, 0.85, 1.0);
    Net_config net_4_pins(10, 20, 60, 4, 10, 0.85, 1.0);
    Net_config net_5_pins(5, 15, 60, 5, 10, 0.85, 1.0);
    Net_config test_conf(10, 20, 60, 3, 10, 0.85, 1.0);
    L.Initialize({4,4}, {{3, 10},{3,10}}, {{5, net_5_pins}, {5, net_4_pins}, {10, net_3_pins}, {30, net_2_pins}});
    //L.Initialize({4,4}, {{3, 10},{3,10}}, {{100, test_conf}});
    //std::cout << L.nets.size() << std::endl;
    L.CheckLegal();
    L.SaveResult("test.txt");

    return 0;
}