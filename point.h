#ifndef _POINT_H_
#define _POINT_H_
#include <iostream>
#include <cmath>

struct Point{
    int x, y, z;
    Point() : x(0), y(0), z(0){}
    Point(int i, int j, int k) : x(i), y(j), z(k){}
    void operator=(const Point& p){
        x = p.x;
        y = p.y;
        z = p.z;
    }
    bool operator==(const Point& p) const{
        return p.x == x && p.y == y && p.z == z;
    }
    bool operator!=(const Point& p) const{
        return p.x != x || p.y != y || p.z != z;
    }
    bool operator<(const Point& p)const{
        return (x < p.x) || (x == p.x && y < p.y) || (x == p.x && y == p.y && z < p.z);
    }
    friend std::ostream& operator<<(std::ostream& os, const Point& p){
        os << "(" << p.x << ", " << p.y << ", " << p.z << ")";
        return os;
    }
    Point operator-(const Point &p) const{
        Point temp;
        temp.x = x - p.x;
        temp.y = y - p.y;
        temp.z = z - p.z;
        return temp;
    }
    Point operator+(const Point &p) const{
        Point temp;
        temp.x = x + p.x;
        temp.y = y + p.y;
        temp.z = z + p.z;
        return temp;
    }
    int manh(){
        return std::abs(x) + std::abs(y) + std::abs(z);
    }
};
#endif
