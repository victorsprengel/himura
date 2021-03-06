#ifndef DEFS_H_
#define DEFS_H_
#include <limits>
#include <queue>
#include <set>
#include <vector>
#include <algorithm>
#include <cstddef>
#include <cassert>
#include <stack>
#include <memory>
#include <deque>
#include <ctime>
#include <map>
#include <math.h>
#include "gurobi_c++.h"
#define MAX_D std::numeric_limits<double>::max()
#define pi 3.14159265358979323846
#define R 6373
#define BETA 0.5
#define FIXED_VARS_GIVE_SAME_PACKAGE_TO_TWO_DIFFERENT_VEHICLES 1
#define NO_PARTITION_CAN_FIT_ANOTHER_PACKAGE 2
#define FOUND_TOUR_DOESNT_RESPECT_WORKING_HOURS 3
#define WORKING_HOURS_CONSTRAINT 0
#define LEAVING_DEPOT_CONSTRAINT 1
#define ANY_DELIVER_COMPLETED_CONSTRAINT 2
#define EVERYTHING_DELIVERED_CONSTRAINT 3
#define FLOW_CONSTRAINT 4
#define VOLUME_CONSTRAINT 5
#define VALUE_CONSTRAINT 6
#define TRUE 1
#define FALSE 0
#define PRINT TRUE
#define PRINT_UB_ERRORS FALSE
#define FREE 0
#define USED 1
#define BLOCKED -1
using namespace std;
using int_pair = pair<int,int>;
using int_triple = tuple<int,int,int>;
using Partition = set<int>;
using Solution = vector<int_triple>;
using SolutionCost = double;
using n3_var = vector<vector<vector<GRBVar>>>;
using n2_var = vector<vector<GRBVar>>;
using n1_var = vector<GRBVar>;
using Edge = int_pair;
using Arc = int_pair;
using Circuit = vector<int>;
using Dicircuit = Circuit;
using Vehicle = int;
using Delivery = int;
using n3_int = vector<vector<vector<int>>>;
using n2_int = vector<vector<int>>;
using n1_int = vector<int>;
template<class T>
using List = vector<T>;
#endif

