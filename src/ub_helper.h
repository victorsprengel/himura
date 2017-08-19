#ifndef VEH_DTO_H
#define VEH_DTO_H
#include <vector>
#include "graph.h"
using namespace std;
using int_pair = pair<int,int>;

struct ub_info {
    vector<bool> in, out;
    vector<double> usedVal, usedVol, usedTime;
    vector<vector<int_pair>> blocked;
    vector<Graph> graph;
    ub_info(const int& n, const int& m);
};

#endif

