#include <iostream>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>

#include "net_config.h"
#include "layout.h"

#define ARGN 10
/**
 * ./main <dir>
 * <test_num>
 * <width> <height> <layers>
 * <obs_num> <min_obs_size> <max_obs_size>
 * <net_num> <pin_num>
 * 
 * ./main 'dir' 10 50 50 3 4 3 10 4 2
 */
int main(int argc, char *argv[]){
    M_Assert(argc == (ARGN + 1), "check main.cpp for args");
    int index = 0;
    const char* directory = argv[++index];
    const int test_num = atoi(argv[++index]);
    const int width = atoi(argv[++index]);
    const int height = atoi(argv[++index]);
    const int layers = atoi(argv[++index]);
    const int obs_num = atoi(argv[++index]);
    const int min_obs_size = atoi(argv[++index]);
    const int max_obs_size = atoi(argv[++index]);
    const int net_num = atoi(argv[++index]);
    const int pin_num = atoi(argv[++index]);

    struct stat st = {0};
    if (stat(directory, &st) == -1) mkdir(directory, 0700);

    std::vector<std::pair<int, Net_config>> net_configs;
    for(int i = 0; i < test_num; ++i){
        net_configs.clear();
        Layout L(width, height, layers, i);
        std::vector<int> obs_nums(layers, obs_num / layers);
        for (int j = 0; j < (obs_num % layers); j++) obs_nums[j]++;
        L.generateObstacles(
            obs_nums, 
            std::vector<std::pair<int, int>>(layers, {min_obs_size, max_obs_size})
        );
        L.autoConfig(net_configs, net_num, pin_num);
        L.generateNets(net_configs);
#ifdef DEBUG
        L.checkLegal();
#endif
        std::string file_name = std::string(directory) + "/" + std::to_string(i) + ".txt";
        L.saveResult(file_name);
        //L.archiveAndReset();
        std::cout << std::endl;
    }
    return 0;
}