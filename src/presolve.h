#ifndef PRESOLVE_H
#define PRESOLVE_H
#include <set>
#include <limits>
#include <algorithm>
#include "input.h"
#include "uf.h"
#define GAMMA 5
#define MAX_D std::numeric_limits<double>::max()
using namespace std;
using int_pair = pair<int,int>;

pair<vector<set<int>>, vector<set<int>>> allowed_variables(const Input& in, const int& n);

#endif

