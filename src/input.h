#ifndef INPUT_H
#define INPUT_H
#include "defs.h"
#include "dist.h"

class Input {
 public:
  vector<double> C, V, M, H, F, E, S, T, J, v, p, lat, lon;
  map<pair<int,int>,double> d;
  Input(char *inVehicles, char *inDeliveries);
  int n, m;
};

#endif

