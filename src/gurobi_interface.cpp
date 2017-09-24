#include "gurobi_interface.h"

static void add_constraint(
    GRBModel& mdl, 
    n3_var& x, 
    n1_var& y, 
    const Input& in, 
    const vector<Partition>& reach, 
    const vector<Partition>& reached, 
    const int& constraint_name) {

  switch (constraint_name) {
    case WORKING_HOURS_CONSTRAINT:
      for (int k = 0; k < in.m; k++) {
        GRBLinExpr e = 0.0;
        for (int i = 0; i <= in.n; i++) {
          for (int j : reach[i]) {
            if (j == in.n + 1) {
              continue;
            }
            e += (x[i][j][k] * (in.T[k] + (in.d.at(int_pair(i,j)) / in.M[k])));
          }
        }
        mdl.addConstr(e, GRB_LESS_EQUAL, in.J[k]);
      }
      break;
    case LEAVING_DEPOT_CONSTRAINT:
      for (int k = 0; k < in.m; k++) {
        GRBLinExpr e = 0.0;
        for (int j : reach[0]) {
          e += x[0][j][k];
        }
        mdl.addConstr(y[k], GRB_EQUAL, e);
      }
      break;
    case ANY_DELIVER_COMPLETED_CONSTRAINT:
      for (int k = 0; k < in.m; k++) {
        for (int i = 0; i <= in.n; i++) {
          for (int j : reach[i]) {
            if (j == in.n + 1) {
              continue;
            }
            mdl.addConstr(x[i][j][k], GRB_LESS_EQUAL, y[k]);
          }
        }
      }
      break;
    case EVERYTHING_DELIVERED_CONSTRAINT:
      for (int j = 1; j <= in.n; j++) {
        GRBLinExpr e = 0.0;
        for (int i : reached[j]) {
          for (int k = 0; k < in.m; k++) {
            e += x[i][j][k];
          }
        }
        mdl.addConstr(e, GRB_EQUAL, 1.0);
      }
      break;
    case FLOW_CONSTRAINT:
      for (int k = 0; k < in.m; k++) {
        for (int i = 1; i <= in.n; i++) {
          GRBLinExpr eLeft = 0.0, eRight = 0.0;

          for (int j : reached[i]) {
            eLeft += x[j][i][k];
          }

          for (int j : reach[i]) {
            eRight += x[i][j][k];
          }

          mdl.addConstr(eLeft, GRB_EQUAL, eRight);
        }
      }
      break;
    case VOLUME_CONSTRAINT:
      for (int k = 0; k < in.m; k++) {
        GRBLinExpr e = 0.0;
        for (int i = 0; i <= in.n; i++) {
          for (int j : reach[i]) {
            if (j == in.n + 1) {
              continue;
            }
            e += (x[i][j][k] * in.v[j]);
          }
        }
        mdl.addConstr(e, GRB_LESS_EQUAL, in.C[k]);
      }
      break;
    case VALUE_CONSTRAINT:
      for (int k = 0; k < in.m; k++) {
        GRBLinExpr e = 0.0;
        for (int i = 0; i <= in.n; i++) {
          for (int j : reach[i]) {
            if (j == in.n + 1) {
              continue;
            }
            e += (x[i][j][k] * in.p[j]);
          }
        }
        mdl.addConstr(e, GRB_LESS_EQUAL, in.V[k]);
      }
      break;
  }
}

void add_constraints(
    GRBModel& mdl, 
    n3_var& x, 
    n1_var& y, 
    const Input& in, 
    const vector<Partition>& reach, 
    const vector<Partition>& reached) {

  for (int i = 0; i < 7; i++) {
    add_constraint(mdl, x, y, in, reach, reached, i);
  }
}

n3_var init_x(
    GRBModel& mdl, 
    const Input& in, 
    const vector<Partition>& reach) {

  n3_var x = n3_var(in.n + 1);
  for (int i = 0; i <= in.n; i++) {
    x[i] = n2_var(in.n + 2);
    for (int j : reach[i]) {
      x[i][j] = n1_var(in.m);
      for (int k = 0; k < in.m; k++) {
        if (j == in.n + 1) {
          x[i][j][k] = mdl.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS);
        } else {
          x[i][j][k] = mdl.addVar(0.0, 1.0, (in.E[k] + in.F[k]*in.d.at(int_pair(i,j)) + (in.H[k]*(in.T[k] + (in.d.at(int_pair(i,j)) / in.M[k])))), GRB_CONTINUOUS);
        }
      }
    }
  }
  return x;
}

n1_var init_y(
    GRBModel& mdl, 
    const Input& in) {

  n1_var y = n1_var(in.m);
  for (int k = 0; k < in.m; k++) {
    y[k] = mdl.addVar(0.0, 1.0, in.S[k], GRB_CONTINUOUS);
  }
  return y;
}

