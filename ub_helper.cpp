#include "ub_helper.h"

ub_info::ub_info(const int& n, const int& m) {
  for (int k = 0; k < m; k++) {
    graph.push_back(Graph(n+2));
    usedVol.push_back(0.0);
    usedVal.push_back(0.0);
    usedTime.push_back(0.0);
    blocked.push_back(vector<int_pair>());
  }
  for (int i = 0; i <= n+1; i++) {
    in.push_back(false);
    out.push_back(false);
  }
}

