#include "net.h"

Net::Net(int id, std::vector<Point> pins_): net_id(id){
   pins = std::move(pins_);
   ori_wl = 0;
}
Net::~Net(){
   //pins.clear();
}