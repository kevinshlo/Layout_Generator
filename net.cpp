#include "net.h"

Net::Net(int id, std::vector<Point> pins_): net_id(id){
   pins = std::move(pins_);
   wl = 0;
}
Net::~Net(){
   //pins.clear();
}

void Net::reset(){
   vias.clear();
   h_segments.clear();
   v_segments.clear();
   wl = 0;
}