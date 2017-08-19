#ifndef PRESOLVE_H
#define PRESOLVE_H
#include <set>
#include <limits>
#include <algorithm>
#include "input.h"
#include "uf.h"
#define GAMMA 10
#define MAX_D std::numeric_limits<double>::max()
using namespace std;
using int_pair = pair<int,int>;

vector<set<int>> get_reach(const input& in, const int& n);

vector<set<int>> get_reached(const vector<set<int>>& reach, const int& n);

void increment_with_mst(vector<set<int>>& reach, vector<set<int>>& reached, const input& in, const int& n);

#endif

