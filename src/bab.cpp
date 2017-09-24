#include "bab.h"

static void fix_vars(
    node_ptr leaf, 
    n3_var& x) {

  while (leaf->parent != nullptr) {
    GRBVar& var = x[leaf->i()][leaf->j()][leaf->k()];
    if (leaf->fixed_value) {
      var.set(GRB_DoubleAttr_LB, 1.0);
      var.set(GRB_DoubleAttr_Start, 1.0);
    } else {
      var.set(GRB_DoubleAttr_UB, 0.0);
      var.set(GRB_DoubleAttr_Start, 0.0);
    }
    leaf = leaf->parent;
  }
}

static void unfix_vars(
    node_ptr leaf, 
    n3_var& x) {

  while (leaf->parent != nullptr) {
    GRBVar& var = x[leaf->i()][leaf->j()][leaf->k()];
    var.set(GRB_DoubleAttr_LB, 0.0);
    var.set(GRB_DoubleAttr_UB, 1.0);
    var.set(GRB_DoubleAttr_Start, GRB_UNDEFINED);
    leaf = leaf->parent;
  }
}

static void update_solution(
    n3_var& x, 
    const int& n, 
    const int& m,
    const vector<Partition>& reach, 
    Solution& sol) {

  sol.clear();
  for (Vehicle k = 0; k < m; k++) {
    for (Delivery i = 0; i <= n; i++) {
      for (Delivery j : reach[i]) {
        if (x[i][j][k].get(GRB_DoubleAttr_X) > 0.99) {
          sol.push_back(make_tuple(i, j, k));
        }
      }
    }
  }
}


//static void copy_solution(
    //Solution& old_sol, 
    //const Solution& new_sol) {

  //old_sol.clear();
  //for (int_triple t : new_sol) {
    //old_sol.push_back(t);
  //}
//}

//static vector<int_triple> fixed_vars(
    //node_ptr current) {

  //vector<int_triple> fixed;

  //while (current->parent != nullptr) {
    //int_triple fixed_var = int_triple(current->i(), current->j(), current->k());

    //if (current->fixed_value) {
      //fixed.push_back(fixed_var);
    //}
  //}

  //return fixed;
//}

//static vector<int_triple> blocked_vars(
    //node_ptr current) {

  //vector<int_triple> blocked;

  //while (current->parent != nullptr) {
    //int_triple fixed_var = int_triple(current->i(), current->j(), current->k());

    //if (!(current->fixed_value)) {
      //blocked.push_back(fixed_var);
    //}
  //}

  //return blocked;
//}

static double solution_cost(
    const Solution& sol, 
    const Input& in) {

  double total = 0.0;
  vector<bool> used = vector<bool>(in.m);
  fill(used.begin(), used.end(), false);
  

  for (int_triple t : sol) {
    Delivery i = get<0>(t);
    Delivery j = get<1>(t);
    Vehicle k = get<2>(t);

    if (j == in.n + 1) {
      continue;
    }

    total += (in.H[k] * in.T[k] + 
              in.E[k] +
              (in.H[k] * (in.d.at(int_pair(i,j)) / in.M[k])) +
              in.F[k] * in.d.at(int_pair(i,j)));

    used[k] = true;
  }

  for (Vehicle k = 0; k < in.m; k++) {
    if (used[k]) {
      total += in.S[k];
    }
  }

  return total;
}

template<typename Container>
static void solve_and_branch(
    Container &col, 
    GRBModel& mdl, 
    n3_var& x, 
    const vector<Partition>& reach, 
    const vector<Partition>& reached,
    double& GUB, 
    Solution& sol, 
    const Input& in) {

  node_ptr current = col.top();
  col.pop();
  fix_vars(current, x);

  if (PRINT) {
    cout << "GLB = " << current->LLB << "    GUB = " << GUB << "    Gap: " << ((GUB - current->LLB) / current->LLB) << endl;
  }

  if (fixed_vars_contain_cycle(current, in.n)) {
    unfix_vars(current, x); return;
  }

  mdl.optimize();
  if (mdl.get(GRB_IntAttr_SolCount) > 0) {
    Dicircuit d = probable_dicircuit(x, in.n, in.m, reach);

    if (!d.empty() && violates_subtour_constraint(d, x, in.n, in.m, reach)) {
      add_subtour_constraint(d, mdl, x, in.n, in.m, reach);
      mdl.optimize();
    }
  }

  if (mdl.get(GRB_IntAttr_SolCount) < 1) {
    unfix_vars(current, x); return;
  }

  current->LLB = max(current->LLB, mdl.get(GRB_DoubleAttr_ObjVal));

  //pair<double,Solution> heuristic_solution = upper_bound(fixed_vars(current), blocked_vars(current), in);
  //double LUB = heuristic_solution.first;
  //if (LUB < GUB) {
    //GUB = LUB;
    //copy_solution(sol, heuristic_solution.second);
  //}

  if (current->LLB >= GUB) { 
    unfix_vars(current, x); return;
  } 

  //covering_constraints(x, mdl, in, reach, reached);
  
  spawn_children(current, reach);

  if (current->has_children()) {
    if (current->has_left_child()) {
      col.push(node_ptr(current->lc));
    }

    if (current->has_right_child()) {
      col.push(node_ptr(current->rc));
    }

  } else {
    GUB = current->LLB;
    update_solution(x, in.n, in.m, reach, sol);
  }

  unfix_vars(current, x);  
}

Solution branch_and_bound(
    GRBModel& mdl, 
    n3_var& x,
    const vector<Partition>& reach,
    const vector<Partition>& reached,
    const Input& in) {

  double GUB = MAX_D;
  auto comparator = [](const node_ptr l, const node_ptr r) { return l->LLB > r->LLB; };
  priority_queue<node_ptr, vector<node_ptr>, decltype(comparator)> pq(comparator);

  pq.push(make_shared<Node>(in.n, in.m, -1, nullptr, -1, 0.0));
  Solution sol;

  while (!pq.empty() && pq.top()->LLB < GUB) {
    solve_and_branch(pq, mdl, x, reach, reached, GUB, sol, in);
  }

  cout << "opt: " << solution_cost(sol, in) << endl;
  assert_viable_solution(sol, in, solution_cost(sol, in));
  return sol;
}
