#include <vector>
#include <queue>
#include <map>
#include <tuple>
#include <iostream>
#include <string>
#include <cmath>
#include <limits>
#include <set>
#include <list>
#include <queue>

#define DEFAULT_COST 1
#define TOL 1e6
#define MAXITERATIONS 1000


bool doubleComparison(double a, double b) {
    if (isinf(a) && isinf(b)) return true;
    return (fabs(a-b) < TOL);
}

struct vec2 {
    uint32_t x, y;
    std::pair<double,double> k;
    double g, rhs, cost;
    bool isEmpty;
    bool onPath;
    bool markForRemoval;
    vec2(uint32_t sx, uint32_t sy) {
        x = sx ; y = sy;
        isEmpty = true;
        onPath = false;
        cost = DEFAULT_COST;
        markForRemoval = false;
    }
    vec2() {
        isEmpty = true;
        onPath = false;
        x = 0; y = 0;
        cost = DEFAULT_COST;
        markForRemoval = false;
    }
    bool operator == (const vec2 &v) const {
        return (x == v.x) && (y == v.y);
    }
    bool operator != (const vec2 &v) const {
        return (x != v.x) && (y != v.y);
    }
    bool operator < (const vec2 &v) const {
        if (k.first + TOL < v.k.first) {
            return true;
        } else if (k.first - TOL > v.k.first) {
            return false;
        } else {
            return k.second < v.k.second;
        }
    }
    bool operator > (const vec2 &v) const {
        if (k.first - TOL > v.k.first) return true;
        else if (k.first - TOL > v.k.first) return false;
        return k.second < v.k.second;
    }


    bool operator <= (const vec2 &v) const {
        return (*this < v) || (*this == v);
    }
    
    bool operator >= (const vec2 &v) const {
        return (*this > v) || (*this == v);
    }
};

typedef std::priority_queue<vec2, std::vector<vec2>, std::greater<vec2> > pqueue;

class DSTAR {
    
    public:
        std::vector<bool> occupancyMap;
        std::vector<vec2> stateMap;
        
        DSTAR(uint32_t width, uint32_t height, vec2 start, vec2 goal);
        void initialize(vec2 start, vec2 goal);
        void updateVertex(vec2 u);
        int computePath();
        bool replan();
        void insert(vec2 u);
        void setOccupied(vec2 u);
        void update(int x, int y, double cost);
        std::list<vec2> path;


    private: 
        int DX[4] = {-1, 0, 1, 0};
        int DY[4] = {0, -1, 0, 1};
        uint32_t height, width;
        uint32_t maxCellSize;
        vec2 start;
        pqueue pq;
        vec2 goal;
        vec2 lastRecordedState;
    
        double k_m;
        
        vec2 calculateKey(vec2 u);
        bool occupied(vec2 u);
        bool isOccupied(vec2 u);
        
        void storeState(vec2 u);
        vec2 getState(uint32_t x, uint32_t y);
        double heuristic(vec2 u, vec2 v);
        void getAdjList(vec2 u, std::list<vec2> &adjList);
        bool findOnList(vec2 u);
        void updatePose(int x, int y);
        void getUnoccupiedAdjlist(vec2 u, std::list<vec2>& adjList);
        void updateCellMap(uint32_t x, uint32_t y, double cost); 
        
};

