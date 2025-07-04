#include <vector>
#include <queue>
#include <map>
#include <tuple>
#include <iostream>
#include <string>
#include <cmath>
#include <limits>
#include <set>




struct vec2 {
    uint32_t x, y;
    std::pair<double,double> k;
    double g, rhs;
    vec2(uint32_t sx, uint32_t sy) {
        x = sx ; y = sy;
    }
};

bool find(uint32_t w, uint32_t h, vec2 u);

class DSTAR {
    
    public:
        std::vector<bool> cells;

        DSTAR(uint32_t width, uint32_t height, vec2 start, vec2 goal);
        void computePath(uint32_t currentX, uint32_t currentY );

    private: 
        std::pair<uint32_t,uint32_t> start;
        std::pair<uint32_t,uint32_t> goal;
        vec2 calculateKey(vec2 u);
        void updateVertex(vec2 u);
        bool occupied(vec2 u);
};

