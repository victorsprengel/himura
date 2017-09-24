#include "subtour_elimination.h"

bool fixed_vars_contain_cycle(
    node_ptr leaf, 
    const int& n) {

  Graph g = Graph(n+2);
  while (leaf->parent != nullptr) {
    if (leaf->fixed_value) {
      g.add_arc(leaf->i(), leaf->j());
    }
    leaf = leaf->parent;
  }
  return g.dicircuit().size();
}

bool violates_subtour_constraint(
    const Dicircuit& dicircuit,
    n3_var& x, 
    const int& n, 
    const int& m, 
    const vector<Partition>& reach) {

  double total = 0.0;

  for (Vehicle k = 0; k < m; k++) {
    for (Delivery i : dicircuit) {
      if (i == n+1) {
        continue;
      }

      for (Delivery j : dicircuit) {
        if (reach[i].find(j) == reach[i].end()) {
          continue;
        }

        total += x[i][j][k].get(GRB_DoubleAttr_X);
      }
    }
  }
  return total > (dicircuit.size() - 1);
}

void add_subtour_constraint(
    const Dicircuit& dicircuit,
    GRBModel& mdl, 
    n3_var& x, 
    const int& n, 
    const int& m, 
    const vector<Partition>& reach) {

  GRBLinExpr e = 0.0;
  for (Vehicle k = 0; k < m; k++) {
    for (Delivery i : dicircuit) {
      if (i == n+1) {
        continue;
      }

      for (Delivery j : dicircuit) {
        if (reach[i].find(j) == reach[i].end()) {
          continue;
        }

        e += x[i][j][k];
      }
    }
  }
  mdl.addConstr(e, GRB_LESS_EQUAL, dicircuit.size() - 1);
}

/* Subtour Elimination Heuristic */
Dicircuit probable_dicircuit(
    n3_var& x, 
    const int& n, 
    const int& m, 
    const vector<Partition>& reach) {

  Graph g = Graph(n+2);
  for (Delivery i = 0; i <= n; i++) {
    for (Delivery j : reach[i]) {
      double sum = 0.0;
      for (Vehicle k = 0; k < m; k++) {
        sum += x[i][j][k].get(GRB_DoubleAttr_X);
      }

      if (sum > ALPHA) {
        g.add_arc(i, j);
      }
    }
  }
  return g.dicircuit();
}
