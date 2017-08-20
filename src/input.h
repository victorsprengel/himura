#ifndef INPUT_H
#define INPUT_H
#include <vector>
#include <map>
#include "gurobi_c++.h"
#include "dist.h"
using namespace std;

class Input {
 public:
  vector<double> C, V, M, H, F, E, S, T, J, v, p;
  map<pair<int,int>,double> d;
  Input(char *inVehicles, char *inDeliveries, int& n, int& m);
};

#endif

