#ifndef INPUT_H
#define INPUT_H
#include <vector>
#include <map>
#include "gurobi_c++.h"
#include "dist.h"
using namespace std;
using pii = pair<int,int>;

class input {
 public:
  vector<double> C, V, M, H, F, E, S, T, J, v, p;
  map<pii,double> d;
  input();
  void read(char *inVehicles, char *inDeliveries, int& n, int& m);
};

#endif

