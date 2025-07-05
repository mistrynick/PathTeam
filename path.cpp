#include "path.hpp"
#include <cassert>


void DSTAR::INITIALIZE(){

    cellHash.clear();
    path.clear();
    openHash.clear();
    while(!openList.empty()) openList.pop();

    km=0;

    s_start.x = sX;
    s_start.y = sY;
    s_goal.x  = gX;
    s_goal.y  = gY;
  
    cellInfo tmp;
    tmp.g = tmp.rhs =  0;
    tmp.cost = C1;
  
    cellHash[s_goal] = tmp;
  
    tmp.g = tmp.rhs = heuristic(s_start,s_goal);
    tmp.cost = C1;
    cellHash[s_start] = tmp;

    s_start = calculateKey(s_start);
  
    s_last = s_start;
  
}


bool DSTAR::find(vec2 u) {
    int hash = u.x * height + u.y;
    assert(hash < cells.size());
    if (cells[hash] || u.cost < 0) {
        return false;
    }
    return true;
}


DSTAR::DSTAR(uint32_t width, uint32_t height, vec2 startPoint, vec2 endPoint) {
    // constructor
}


int main(int argc, char ** argv) {
    uint32_t sx, sy;
    std::cin >> sx >> sy;
    vec2 startingPoint(sx,sy);
    std::cin >> sx >> sy;
    vec2 endPoint(sx,sy);
    int numberOfObstacles;
    std::cin >> numberOfObstacles;

    DSTAR dstar(10, 10, startingPoint, endPoint);

    return 0;
}
