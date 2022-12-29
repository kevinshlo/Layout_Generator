#ifndef _NET_CONFIG_H
#define _NET_CONFIG_H
struct Net_config{
   Net_config(size_t _min_wl, size_t _max_wl, size_t _wl_limit, int _pin_num, int _reroute_num, float _momentum1, float _momentum2):
      min_wl(_min_wl), max_wl(_max_wl), wl_limit(_wl_limit), pin_num(_pin_num), reroute_num(_reroute_num), momentum1(_momentum1), momentum2(_momentum2){

      }
   Net_config(size_t _min_wl, size_t _max_wl, size_t _wl_limit, int _pin_num, int _reroute_num, float _momentum1):
      min_wl(_min_wl), max_wl(_max_wl), wl_limit(_wl_limit), pin_num(_pin_num), reroute_num(_reroute_num), momentum1(_momentum1), momentum2(1.0){
         
      }
   ~Net_config(){}
   size_t min_wl;
   size_t max_wl;
   size_t wl_limit;
   int pin_num;
   int reroute_num;
   float momentum1;
   float momentum2;
};
#endif