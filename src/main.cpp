#include <ctime>
#include "input.h"
#include "presolve.h"
#include "bab.h"
#include "gurobi_c++.h"
#include "debug.h"
#include <limits>
#define MAX_D std::numeric_limits<double>::max()
using namespace std;

static x_vars init_x(const int& n, const int& m, GRBModel& mdl, const Input& in, const vector<set<int>>& reach) {
  x_vars x = x_vars(n+1);
  for (int i = 0; i <= n; i++) {
    x[i] = vector<vector<GRBVar>>(n+2);
    for (int j : reach[i]) {
      x[i][j] = vector<GRBVar>(m);
      for (int k = 0; k < m; k++) {
        if (j == n+1)
          x[i][j][k] = mdl.addVar(0.0, 1.0, 0.0, GRB_CONTINUOUS);
        else
          x[i][j][k] = mdl.addVar(0.0, 1.0, (in.E[k] + in.F[k]*in.d.at(pair<int,int>(i,j)) + (in.H[k]*(in.T[k] + (in.d.at(pair<int,int>(i,j)) / in.M[k])))), GRB_CONTINUOUS);
      }
    }
  }
  return x;
}

static y_vars init_y(const int& m, GRBModel& mdl, const Input& in) {
  y_vars y = y_vars(m);
  for (int k = 0; k < m; k++) {
    y[k] = mdl.addVar(0.0, 1.0, in.S[k], GRB_CONTINUOUS);
  }
  return y;
}

static void every_package_has_to_be_delivered_by_exactly_one_vehicle(GRBModel& mdl, x_vars& x, const int& n, const int& m, const vector<set<int>>& reached) {
  for (int j = 1; j <= n; j++) {
    GRBLinExpr e = 0.0;
    for (int i : reached[j])
      for (int k = 0; k < m; k++)
        e += x[i][j][k];
    mdl.addConstr(e, GRB_EQUAL, 1.0);
  }
}

static void every_vehicle_has_to_obey_its_volume_capacity(GRBModel& mdl, x_vars& x, const int& n, const int& m, const Input& in, const vector<set<int>>& reach) {
  for (int k = 0; k < m; k++) {
    GRBLinExpr e = 0.0;
    for (int i = 0; i <= n; i++) {
      for (int j : reach[i]) {
        if (j == n+1)
          continue;
        e += (x[i][j][k] * in.v[j]);
      }
    }
    mdl.addConstr(e, GRB_LESS_EQUAL, in.C[k]);
  }
}

static void every_vehicle_has_to_obey_its_value_capacity(GRBModel& mdl, x_vars& x, const int& n, const int& m, const Input& in, const vector<set<int>>& reach) {
  for (int k = 0; k < m; k++) {
    GRBLinExpr e = 0.0;
    for (int i = 0; i <= n; i++) {
      for (int j : reach[i]) {
        if (j == n+1)
          continue;
        e += (x[i][j][k] * in.p[j]);
      }
    }
    mdl.addConstr(e, GRB_LESS_EQUAL, in.V[k]);
  }
}

static void the_number_of_vehicles_arriving_at_one_delivery_is_the_same_as_the_number_leaving(GRBModel& mdl, x_vars& x, const int& n, const int& m, const vector<set<int>>& reach, const vector<set<int>>& reached) {
  for (int k = 0; k < m; k++) {
    for (int i = 1; i <= n; i++) {
      GRBLinExpr eLeft = 0.0, eRight = 0.0;

      for (int j : reached[i])
        eLeft += x[j][i][k];

      for (int j : reach[i]) 
        eRight += x[i][j][k];

      mdl.addConstr(eLeft, GRB_EQUAL, eRight);
    }
  }
}

static void every_vehicle_leaving_the_depot_has_to_be_used(GRBModel& mdl, x_vars& x, y_vars& y, const int& m, const vector<set<int>>& reach) {
  for (int k = 0; k < m; k++) {
    GRBLinExpr e = 0.0;
    for (int j : reach[0])
      e += x[0][j][k];
    mdl.addConstr(y[k], GRB_EQUAL, e);
  }
}

static void every_vehicle_that_delivered_something_has_to_be_used(GRBModel& mdl, x_vars& x, y_vars& y, const int& n, const int& m, const vector<set<int>>& reach) {
  for (int k = 0; k < m; k++) {
    for (int i = 0; i <= n; i++) {
      for (int j : reach[i]) {
        if (j == n+1)
          continue;
        mdl.addConstr(x[i][j][k], GRB_LESS_EQUAL, y[k]);
      }
    }
  }
}

static void every_vehicle_has_to_respect_working_hours(GRBModel& mdl, x_vars& x, const int& n, const int& m, const Input& in, const vector<set<int>>& reach) {
  for (int k = 0; k < m; k++) {
    GRBLinExpr e = 0.0;
    for (int i = 0; i <= n; i++) {
      for (int j : reach[i]) {
        if (j == n+1)
          continue;
        e += (x[i][j][k] * (in.T[k] + (in.d.at(pair<int,int>(i,j)) / in.M[k])));
      }
    }
    mdl.addConstr(e, GRB_LESS_EQUAL, in.J[k]);
  }
}

static void add_constraints(GRBModel& mdl, x_vars& x, y_vars& y, const int& n, const int& m, const Input& in, const vector<set<int>>& reach, const vector<set<int>>& reached) {
  every_package_has_to_be_delivered_by_exactly_one_vehicle(mdl, x, n, m, reached);
  every_vehicle_has_to_obey_its_volume_capacity(mdl, x, n, m, in, reach);
  every_vehicle_has_to_obey_its_value_capacity(mdl, x, n, m, in, reach);
  the_number_of_vehicles_arriving_at_one_delivery_is_the_same_as_the_number_leaving(mdl, x, n, m, reach, reached);
  every_vehicle_leaving_the_depot_has_to_be_used(mdl, x, y, m, reach);
  every_vehicle_that_delivered_something_has_to_be_used(mdl, x, y, n, m, reach);
  every_vehicle_has_to_respect_working_hours(mdl, x, n, m, in, reach);
}

int main(int argc, char** argv) {
  if (argc != 3) {
    cout << "./router vehicle.data package.data" << endl;
    return 1;
  }

  srand(time(NULL));
  int n, m;
  Input in = Input(argv[1], argv[2], n, m);
  cout << "n = " << n << "    m = " << m << endl;
  pair<vector<set<int>>, vector<set<int>>> allowed_vars = allowed_variables(in, n);

  try {
    GRBEnv env = GRBEnv();
    GRBModel mdl = GRBModel(env);
    auto x = init_x(n, m, mdl, in, allowed_vars.first);
    auto y = init_y(m, mdl, in);
    mdl.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    mdl.set(GRB_IntParam_OutputFlag, 0);
    add_constraints(mdl, x, y, n, m, in, allowed_vars.first, allowed_vars.second);

    vector<triple> solution;
    double optimal_value =  branch_and_bound(mdl, x, n, m, solution, allowed_vars.first, allowed_vars.second, in);
    cout << "optimal value: " << optimal_value << endl;

    if (abs(MAX_D - optimal_value) > 1e8) {
      assert_viable_solution(solution, n, m, in, optimal_value);
    }
  } catch (GRBException e) {
    cout << e.getMessage() << endl;
  }

  return 0;
}
