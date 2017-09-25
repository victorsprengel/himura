#include "defs.h"
#include "input.h"
#include "presolve.h"
#include "gurobi_interface.h"
#include "bab.h"

static void check_arguments(
    const int& argc) {

  if (argc != 3) {
    cout << "usage: ./router vehicle.data package.data" << endl;
    exit(0);
  }
}

int main(int argc, char** argv) {
  check_arguments(argc);
  srand(time(NULL));

  Input in = Input(argv[1], argv[2]);
  cout << "n = " << in.n << "    m  = " << in.m << endl;
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

  } catch (GRBException e) {
    cout << e.getMessage() << endl;
  }

  return 0;
}
