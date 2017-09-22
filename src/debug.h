#ifndef DEBUG_H_
#define DEBUG_H_
#include "input.h"
#include "graph.h"
#include <vector>
using namespace std;
using triple = tuple<int, int, int>;

void assert_viable_solution(const vector<triple>& sol, const int& n, const int& m, const Input& in, const double& expected_val);

#endif
