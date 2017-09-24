#ifndef GUROBI_INTERFACE_H_
#define GUROBI_INTERFACE_H_
#include "defs.h"
#include "input.h"

void add_constraints(
    GRBModel& mdl, 
    n3_var& x, 
    n1_var& y, 
    const Input& in, 
    const vector<Partition>& reach, 
    const vector<Partition>& reached);

n3_var init_x(
    GRBModel& mdl, 
    const Input& in, 
    const vector<Partition>& reach);

n1_var init_y(
    GRBModel& mdl, 
    const Input& in);

#endif
