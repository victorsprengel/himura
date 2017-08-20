#ifndef BAB_H
#define BAB_H
#include <stack>
#include <limits>
#include <queue>
#include <set>
#include "node.h"
#include "input.h"
#include "graph.h"
#include "gurobi_c++.h"
#include "heuristics.h"
#define MAX_D std::numeric_limits<double>::max()
#define PRINT_FREQ 100
using namespace std;
using x_vars = vector<vector<vector<GRBVar>>>;
using y_vars = vector<GRBVar>;
using triple = tuple<int,int,int>;
using node_ptr = shared_ptr<Node>;

double branch_and_bound(GRBModel& mdl, x_vars& x, const int& n, const int& m, vector<triple>& sol, const vector<set<int>>& reach, const vector<set<int>>& reached, const Input& in);

#endif

