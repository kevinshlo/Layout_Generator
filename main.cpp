#include <iostream>
#include "layout.h"

int main(){
    Layout L(50, 50, 2);
    int width = 1;
    int height = 1;
    int counter = 0;
    int obs_num = randInt(2, 4);
    while(counter < obs_num){
        width = randInt(2, 8);
        int x = randInt(0, 50 - width - 1);
        int y = randInt(0, 50 - height - 1);
        Point p1(x, y, 0);
        Point p2(x + width, y + height, 0);
        if(L.AddObstacle(p1, p2)){
            counter++;
        }
    }
    obs_num = randInt(2, 4);
    width = 1;
    counter = 0;
    while(counter < obs_num){
        height = randInt(2, 8);
        int x = randInt(0, 50 - width - 1);
        int y = randInt(0, 50 - height - 1);
        Point p1(x, y, 1);
        Point p2(x + width, y + height, 1);
        if(L.AddObstacle(p1, p2)){
            counter++;
        }
    }
    //L.GenerateNet({5, 10}, 60, 1.0, 5, 17);
    //L.SaveResult("test_0.txt");
    
    //std::cout << "         " << std::endl;
    for(int i = 0; i < 20; ++i){
        if(L.GenerateNet({10, 25}, 60, 1.0, 5, i)){
            std::cout << "Create net " << i << " success" << std::endl;
        }else{
            std::cout << "Create net " << i << " fail" << std::endl;
        }
    }
    
    
    L.SaveResult("test.txt");
    
    return 0;
}