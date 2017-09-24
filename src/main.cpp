#include "defs.h"
#include "input.h"
#include "presolve.h"
#include "gurobi_interface.h"
#include "bab.h"
#include "debug.h"

static void check_arguments(
    const int& argc) {

  if (argc != 3) {
    cout << "usage: ./router vehicle.data package.data" << endl;
    exit(0);
  }
}

static double solution_cost(
    const Solution& sol, 
    const Input& in) {

  double total = 0.0;
  vector<bool> used = vector<bool>(in.m);
  fill(used.begin(), used.end(), false);
  

  for (int_triple t : sol) {
    int i = get<0>(t);
    int j = get<1>(t);
    int k = get<2>(t);

    if (j == in.n + 1) {
      continue;
    }

    total += (in.H[k] * in.T[k] + 
              in.E[k] +
              (in.H[k] * (in.d.at(int_pair(i,j)) / in.M[k])) +
              in.F[k] * in.d.at(int_pair(i,j)));

    used[k] = true;
  }

  for (int k = 0; k < in.m; k++) {
    if (used[k]) {
      total += in.S[k];
    }
  }

  return total;
}

int main(int argc, char** argv) {
  check_arguments(argc);
  srand(time(NULL));

  Input in = Input(argv[1], argv[2]);
  vector<Partition> reach = find_reach(in);
  vector<Partition> reached = reached_from_reach(reach, in.n);

  try {
    GRBEnv env = GRBEnv();
    GRBModel mdl = GRBModel(env);
    mdl.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
    mdl.set(GRB_IntParam_OutputFlag, 0);

    n3_var x = init_x(mdl, in, reach);
    n1_var y = init_y(mdl, in);
    add_constraints(mdl, x, y, in, reach, reached);

    Solution sol = branch_and_bound(mdl, x, reach, reached, in);
    cout << "opt: " << solution_cost(sol, in) << endl;

    assert_viable_solution(sol, in, solution_cost(sol, in));
  } catch (GRBException e) {
    cout << e.getMessage() << endl;
  }

  return 0;
}
