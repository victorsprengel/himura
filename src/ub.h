#ifndef UB_H_
#define UB_H_
#include "defs.h"
#include "input.h"
#include "node.h"
#include "graph.h"
#define XI 1.2

Solution heuristic_solution(const List<int_triple>& used, const List<int_triple>& blocked, const vector<Partition>& reach, const Input& in);

#endif
