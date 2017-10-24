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

static Solution sorted_sol(const Solution& sol, const int& m) {
  Solution sorted;
  for (Vehicle k = 0; k < m; k++) {
    Delivery current = 0;

    for ( ; ; ) {
      int index_of_edge = -1;

      for (size_t j = 0; j < sol.size(); j++) {
        if (get<0>(sol[j]) == current && get<2>(sol[j]) == k) {
          index_of_edge = j;
        }
      }

      if (index_of_edge == -1) {
        break;
      }

      sorted.push_back(sol[index_of_edge]);
      current = get<1>(sol[index_of_edge]);
    }
    
  }

  return sorted;
}

int main(int argc, char** argv) {
  check_arguments(argc);
  srand(time(NULL));
  cout.precision(10);

  Input in = Input(argv[1], argv[2]);
  cout << "n = " << in.n << "    m  = " << in.m << "    t = " << TOLERANCE << endl;
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
    for (int_triple t : sorted_sol(sol, in.m)) {
      if (get<1>(t) == in.n + 1) {
        continue;
      }
      cout << in.lat[get<0>(t)] << "," << in.lon[get<0>(t)] << " " << in.lat[get<1>(t)] << "," << in.lon[get<1>(t)] <<  " " << get<2>(t) << endl;
    }

  } catch (GRBException e) {
    cout << e.getMessage() << endl;
  }

  return 0;
}
