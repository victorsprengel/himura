#ifndef SUBTOUR_ELIMINATION_H_
#define SUBTOUR_ELIMINATION_H_
#include "defs.h"
#include "graph.h"
#include "node.h"

bool fixed_vars_contain_cycle(node_ptr leaf, const int& n);

bool violates_subtour_constraint(const Dicircuit& dicircuit, n3_var& x, const int& n, const int& m, const vector<Partition>& reach);

void add_subtour_constraint(const Dicircuit& dicircuit, GRBModel& mdl, n3_var& x, const int& n, const int& m, const vector<Partition>& reach);

Dicircuit probable_dicircuit(n3_var& x, const int& n, const int& m, const vector<Partition>& reach);

#endif
