#include "path.hpp"





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
