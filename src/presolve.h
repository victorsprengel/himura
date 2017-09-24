#ifndef PRESOLVE_H
#define PRESOLVE_H
#include "input.h"
#include "defs.h"
#include "graph.h"

vector<Partition> find_reach(const Input& in);

vector<Partition> reached_from_reach(const vector<Partition>& reach, const int& n);

#endif

