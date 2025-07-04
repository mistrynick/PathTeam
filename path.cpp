#include "path.hpp"


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
