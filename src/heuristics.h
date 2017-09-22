#ifndef HEURISTICS_H
#define HEURISTICS_H
#include <limits>
#include <set>
#include <cassert>
#include <algorithm>
#include <memory>
#include <stack>
#include "gurobi_c++.h"
#include "graph.h"
#include "node.h"
#include "input.h"
#include "uf.h"
#include "ll_node.h"
#include "debug.h"
#define MAX_D std::numeric_limits<double>::max()
#define ALPHA 0.7
#define BETA 0.5
using namespace std;
using triple = tuple<int,int,int>;
using x_vars = vector<vector<vector<GRBVar>>>;
using y_vars = vector<GRBVar>;
using int_pair = pair<int,int>;
using node_ptr = shared_ptr<Node>;

vector<int> subtour_elimination_heuristic(x_vars& x, const int& n, const int& m, const vector<set<int>>& reach);

void covering_constraints(x_vars& x, GRBModel& model, const int& n, const int& m, const Input& in, const vector<set<int>>& reach, const vector<set<int>>& reached);

pair<double, vector<triple>> upper_bound(node_ptr& current, const Input& in);

#endif

