#ifndef HEURISTICS_H
#define HEURISTICS_H
#include "graph.h"
#include "input.h"
#include "defs.h"
#include "node.h"
#include "debug.h"

vector<int> subtour_elimination_heuristic(n3_var& x, const int& n, const int& m, const vector<set<int>>& reach);

void covering_constraints(n3_var& x, GRBModel& model, const Input& in, const vector<Partition>& reach, const vector<Partition>& reached);

pair<double, Solution> upper_bound(const vector<int_triple>& used, const vector<int_triple>& blocked, const Input& in);

#endif

