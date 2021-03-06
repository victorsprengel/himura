#ifndef BAB_H
#define BAB_H
#include "defs.h"
#include "node.h"
#include "graph.h"
#include "subtour_elimination.h"
#include "input.h"
#include "debug.h"
#include "ub.h"
#define TOLERANCE 0.4

Solution branch_and_bound(
    GRBModel& mdl, 
    n3_var& x, 
    const vector<Partition>& reach, 
    const vector<Partition>& reached, 
    const Input& in);

#endif

