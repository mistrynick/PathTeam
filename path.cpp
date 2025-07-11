#include "path.hpp"
#include <cassert>

bool DSTAR::isOccupied(vec2 u) {
    int hash = u.x * height + u.y;
    assert(hash < maxCellSize);
    if (occupancyMap[hash] || u.cost < 0) {
        return true;
    }
    return false;
}

vec2 DSTAR::getState(uint32_t x, uint32_t y) {
    int hash = x * height + y;
    assert(hash < maxCellSize);
    return stateMap[hash];
}

void DSTAR::setOccupied(vec2 u) {
    int hash = u.x * height + u.y;
    assert(hash < maxCellSize);
    occupancyMap[hash] = true;
}

void DSTAR::storeState(vec2 u) {
    int hash = u.x * height + u.y;
    assert(hash < maxCellSize);
    stateMap[hash] = u;
}

double DSTAR::heuristic(vec2 u, vec2 v) {
    double distance = (v.x - u.x) * (v.x - u.x);
    distance += (v.y - u.y) * (v.y - u.y);
    distance = std::sqrt(distance);
    return distance;

}

vec2 DSTAR::calculateKey(vec2 u) {
    
    double value = fmin(u.rhs, u.g);
    u.k.first = value + heuristic(u, start);
    u.k.second = value;
    return u;
}

void DSTAR::updateCellMap(uint32_t x, uint32_t y, double cost) {
    vec2 state(x,y);
    state.cost = cost;
    storeState(state);
    updateVertex(state);
}

void DSTAR::initialize(vec2 sp, vec2 ep) {
    k_m = 0;
    start = sp; goal = ep;
    start.cost = 1;
    goal.g = goal.rhs = 0;
    goal.cost = 1;
    storeState(goal);
    start.g = start.rhs = heuristic(start, goal);
    start.cost = 1;
    lastRecordedState = start;
    storeState(start);
    for (uint32_t i = 0; i < width; i++) {
        for (uint32_t j = 0; j < height; j++) {
            vec2 state(i, j);
            state.cost = 1.0; // Default cost
            state.g = state.rhs = heuristic(state, goal);
            storeState(state);
        }
    }
}

void DSTAR::updateVertex(vec2 u) {
    std::list<vec2> adjList;
    if (u != goal) {
        getAdjList(u, adjList);
        double initialCost = INFINITY;
        for (vec2 v: adjList) {
            double newCost = v.g + v.cost;
            initialCost = std::min(initialCost, newCost);
        }
        if (!doubleComparison(u.rhs, initialCost)) {
            u.rhs = initialCost;
        }
    }
    if (!doubleComparison(u.g, u.rhs)) {
        insert(u);
    }
}
void DSTAR::updatePose(int x, int y) {
    start.x = x;
    start.y = y;
    k_m += heuristic(lastRecordedState, start);
    storeState(start);
    start = calculateKey(start);
    lastRecordedState = start;
}

bool DSTAR::replan() {
    path.clear();
    int resultFromPathComp = computePath();
    if (resultFromPathComp < 0) { std::cerr << "Cannot compute Path! computePath failed\n"; return false; }
    std::list<vec2> n;
    vec2 cur = start;
    if (isinf(start.g)) {std::cerr<< "Cannot compute Path! start.g is infinity\n"; return false;}

    while (cur != goal)
    {
       
       
        path.push_back(cur);
       // getAdjList(cur, n);
        getUnoccupiedAdjlist(cur, n);
        
        if (n.empty()) {
            std::cerr<< "Cannot compute Path! n is empty\n"; return false;
        }
        double cmin = INFINITY;
        double tmin;
        vec2 smin;
        for (vec2& it: n) {
            double val = heuristic(cur, it); // using heuristic as a cost
            double distanceCost = heuristic(it, goal) + heuristic(start, it);
            val += it.g;
            if (doubleComparison(val, cmin)) {
                if (tmin > distanceCost) {
                    tmin = distanceCost;
                    cmin = val;
                    smin = it;
                }
            } else if (val < cmin) {
                tmin = distanceCost;
                cmin = val;
                smin = it;
            }
        }
        n.clear();
        setOccupied(cur);
        cur = smin;
        
        if (cur == lastRecordedState) {break;}
        if (cur.x != goal.x && cur.y != goal.y) {
            updatePose(cur.x, cur.y);
        }
        
    }
    path.push_back(goal);
    return true;
}

void DSTAR::insert(vec2 u) {
    u.onPath = true;
    storeState(u);
    pq.push(u);
}

void DSTAR::getAdjList(vec2 u, std::list<vec2>& adjList) {
    
    adjList.clear();
    u.k.first = -1; u.k.second = -1;
    if (isOccupied(u)) {
        std::cerr << "state is occupied!!" << std::endl;
        return;
    }
    for (int i=0;i<4;i++) {
        int nx = u.x + DX[i];
        int ny = u.y + DY[i];
        if (nx > 0 && nx <= width && ny > 0 && ny <= height) {
            if (getState(nx, ny).markForRemoval) {continue;}
            adjList.push_front(vec2(u.x+DX[i], u.y+DY[i]));
        }
        
    }
    
}

void DSTAR::getUnoccupiedAdjlist(vec2 u, std::list<vec2>& adjList) {
    adjList.clear();
    u.k.first = -1; u.k.second = -1;
    for (int i=0;i<4;i++) {
        uint32_t nx = u.x+DX[i];
        uint32_t ny = u.y+DY[i];
        if (isOccupied(vec2(nx, ny))) continue;
        if (nx > 0 && nx <= width && ny > 0 && ny <= height) {
            if (getState(nx, ny).markForRemoval) {continue;}
            adjList.push_front(vec2(u.x+DX[i], u.y+DY[i]));
        }
    }
}

void DSTAR::update(int x, int y, double cost) {
    vec2 state(x,y);
    state.cost = cost;
    storeState(state);
}

bool DSTAR::findOnList(vec2 u) {
    vec2 state = getState(u.x, u.y);
    return state.onPath;
}

DSTAR::DSTAR(uint32_t width, uint32_t height, vec2 startPoint, vec2 endPoint) {
    // constructor
    this->width = width; this->height = height; 
    maxCellSize = (width * height) + width+1;
    start = startPoint; goal = startPoint;
    occupancyMap.resize(width * height, false);
    stateMap.resize(width * height, vec2());
}



int DSTAR::computePath() {
    std::list<vec2> s;
    if (pq.empty()) return 1;
    int steps = 0;
    start = calculateKey(start);
    while (!pq.empty() && (pq.top() < start || start.rhs != start.g))
    {
        if (steps++ > MAXITERATIONS) {
            return -1;
        }
        vec2 u;
        bool earlyTermination = start.rhs != start.g;
        while (true)
        {
            if (pq.empty()) return 1;
            u = pq.top(); pq.pop();
            if (!findOnList(u)) continue;
            if (!(u < start) && (!earlyTermination)) return 2;
            break;
        }
        vec2 cur = getState(u.x, u.y);
        cur.markForRemoval = true;
        storeState(cur);
        vec2 k_old = u;
        if (k_old < calculateKey(u)) {
            insert(u);
        } else if (u.g > u.rhs) {
            u.g = u.rhs;
            getUnoccupiedAdjlist(u, s);
            for (vec2 a: s) {
                updateVertex(a);
            }
        } else {
            u.g = INFINITY;
            getUnoccupiedAdjlist(u, s);
            for (vec2& state: s) {
                updateVertex(state);
            }
            updateVertex(u);
        }
    }
    return 0;
}


int main(int argc, char ** argv) {
    uint32_t sx, sy;
    std::cin >> sx >> sy;
    vec2 startingPoint(sx,sy);
    std::cin >> sx >> sy;
    vec2 endPoint(sx,sy);
    int numberOfObstacles;
    std::cin >> numberOfObstacles;
    int width = 100;
    int height = 100;

    DSTAR dstar(width, height, startingPoint, endPoint);
    for (int i=0;i<width;i++) {
        for (int j=0;j<height;j++) {
            dstar.insert(vec2(i, j));
        }
    }
    for (int i=0;i<numberOfObstacles;i++) {
        int x, y;
        std::cin >> x >> y;
        vec2 state(x,y);
        dstar.setOccupied(vec2(x, y));
        dstar.update(x,y, -1);
    }
    
    dstar.initialize(startingPoint, endPoint);
    //int res = dstar.computePath();
    //std::cout << res << std::endl;
    dstar.replan();
    for (auto& it: dstar.path) {
        std::cout << it.x << " " << it.y << std::endl;
    }

    return 0;
}
