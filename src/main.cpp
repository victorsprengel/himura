#include <ctime>
#include "input.h"
#include "presolve.h"
#include "bab.h"
#include "graph.h"
#include "gurobi_c++.h"
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

static void add_constraints(GRBModel& mdl, x_vars& x, y_vars& y, const int& n, const int& m, const Input& in, const vector<set<int>>& reach, const vector<set<int>>& reached) {
  for (int j = 1; j <= n; j++) {
    GRBLinExpr e = 0.0;
    for (int i : reached[j])
      for (int k = 0; k < m; k++)
        e += x[i][j][k];
    mdl.addConstr(e, GRB_EQUAL, 1.0);
  }

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

  for (int k = 0; k < m; k++) {
    GRBLinExpr e = 0.0;
    for (int j : reach[0])
      e += x[0][j][k];
    mdl.addConstr(y[k], GRB_EQUAL, e);
  }

  for (int k = 0; k < m; k++) {
    for (int i = 0; i <= n; i++) {
      for (int j : reach[i]) {
        if (j == n+1)
          continue;
        mdl.addConstr(x[i][j][k], GRB_LESS_EQUAL, y[k]);
      }
    }
  }

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

static void assert_optimal_solution(const vector<triple>& sol, const int& n, const int& m, const Input& in, const double& optVal) {
  for (int j = 1; j <= n; j++) {
    int delivered = 0;
    for (triple t : sol)
      if (get<1>(t) == j)
        delivered++;
    assert(delivered == 1);
  }

  for (int k = 0; k < m; k++) {
    double vol = 0.0;
    for (triple t : sol)
      if (get<2>(t) == k && get<1>(t) != n+1)
        vol += in.v[get<1>(t)];
    assert(vol <= in.C[k]);
  }


  for (int k = 0; k < m; k++) {
    double val = 0.0;
    for (triple t : sol)
      if (get<2>(t) == k && get<1>(t) != n+1)
        val += in.p[get<1>(t)];
    assert(val <= in.V[k]);
  }

  for (int k = 0; k < m; k++) {
    for (int j = 1; j <= n; j++) {
      int got_in = 0, got_out = 0;
      for (triple t : sol) {
        if (get<2>(t) == k) {
          if (get<0>(t) == j) {
            got_out++;
          }
          if (get<1>(t) == j) {
            got_in++;
          }
        }
      }
      assert(got_in == got_out && got_in <= 1 && got_out <= 1);
    }
  }

  for (int k = 0; k < m; k++) {
    double time = 0.0;
    for (triple t : sol) {
      if (get<2>(t) == k && get<1>(t) != n+1) {
        time += (in.T[k] + (in.d.at(int_pair(get<0>(t),get<1>(t))) / in.M[k]));
      }
    }
    assert(time <= in.J[k]);
  }

  for (int k = 0; k < m; k++) {
    Graph g = Graph(n+2);
    for (triple t : sol)
      if (get<2>(t) == k)
        g.add_arc(get<0>(t), get<1>(t));
    vector<int> cycle = g.tour();
    assert (cycle.size() == 0);
  }

  double solution = 0.0;
  for (int k = 0; k < m; k++) {
    bool used = false;
    for (triple t : sol)
      if (get<2>(t) == k && get<1>(t) != n+1)
        used = true;
    if (used)
      solution += in.S[k];
  }
  for (triple t : sol) {
    int i = get<0>(t);
    int j = get<1>(t);
    int k = get<2>(t);
    assert (i != j);
    assert (j != 0);
    assert (i != n+1);
    if (j == n+1)
      continue;
    solution += (in.H[k] * in.T[k]);
    solution += ((in.H[k] * in.d.at(int_pair(i,j))) / in.M[k]);
    solution += (in.E[k]);
    solution += (in.F[k] * in.d.at(int_pair(i,j)));
  }

  assert(abs(solution - optVal) <= 1e-8);
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
      assert_optimal_solution(solution, n, m, in, optimal_value);
    }
  } catch (GRBException e) {
    cout << e.getMessage() << endl;
  }

  return 0;
}
