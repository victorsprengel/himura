#include "bab.h"

static bool fixed_vars_contain_cycle(node_ptr leaf, const int& n) {
  Graph g = Graph(n+2);
  while (leaf->parent != nullptr) {
    if (leaf->fixed_value) {
      g.add_arc(leaf->i(), leaf->j());
    }
    leaf = leaf->parent;
  }
  return g.tour().size();
}

static void fix_vars(node_ptr leaf, x_vars& x) {
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

static void unfix_vars(node_ptr leaf, x_vars& x) {
  while (leaf->parent != nullptr) {
    GRBVar& var = x[leaf->i()][leaf->j()][leaf->k()];
    var.set(GRB_DoubleAttr_LB, 0.0);
    var.set(GRB_DoubleAttr_UB, 1.0);
    var.set(GRB_DoubleAttr_Start, GRB_UNDEFINED);
    leaf = leaf->parent;
  }
}

static void update_solution(x_vars& x, const int& n, const int& m, const vector<set<int>>& reach, vector<triple>& sol) {
  sol.clear();
  for (int k = 0; k < m; k++)
    for (int i = 0; i <= n; i++)
      for (int j : reach[i])
        if (x[i][j][k].get(GRB_DoubleAttr_X) > 0.99)
          sol.push_back(make_tuple(i, j, k));
}

static bool violates_subtour_constraint(const vector<int>& cycle, x_vars& x, const int& n, const int& m, const vector<set<int>>& reach) {
  double total = 0.0;

  for (int k = 0; k < m; k++) {
    for (int i : cycle) {
      if (i == n+1)
        continue;
      for (int j : cycle) {
        if (reach[i].find(j) == reach[i].end())
          continue;
        total += x[i][j][k].get(GRB_DoubleAttr_X);
      }
    }
  }
  return total > (cycle.size() - 1);
}

static void add_subtour_constraint(GRBModel& mdl, x_vars& x, const vector<int>& cycle, const int& n, const int& m, const vector<set<int>>& reach) {
  GRBLinExpr e = 0.0;
  for (int k = 0; k < m; k++) {
    for (int i : cycle) {
      if (i == n+1)
        continue;
      for (int j : cycle) {
        if (reach[i].find(j) == reach[i].end())
          continue;
        e += x[i][j][k];
      }
    }
  }
  mdl.addConstr(e, GRB_LESS_EQUAL, cycle.size() - 1);
}

static void copy_solution(vector<triple>& old_sol, const vector<triple>& new_sol) {
  old_sol.clear();
  for (triple t : new_sol)
    old_sol.push_back(t);
}

template<typename Container>
static void solve_and_branch(Container &col , GRBModel& mdl, x_vars& x, const int& n, 
                             const int& m, const vector<set<int>>& reach, 
                             const vector<set<int>>& reached,
                             double& GUB, vector<triple>& sol, bool print,
                             const Input& in) {
  static int it = 0;
  node_ptr current = col.top();
  col.pop();
  fix_vars(current, x);

  if (print && (it % PRINT_FREQ) == 1)
    cout << "GLB = " << current->LLB << "    GUB = " << GUB << "    Gap: " << ((GUB / current->LLB) - 1.0) << "    (it = " << it << ")" << endl;
  it++;

  if (fixed_vars_contain_cycle(current, n)) {
    unfix_vars(current, x); return;
  }

  mdl.optimize();
  if (mdl.get(GRB_IntAttr_SolCount) > 0) {
    vector<int> cycle = subtour_elimination_heuristic(x, n, m, reach);
    if (!cycle.empty() && violates_subtour_constraint(cycle, x, n, m, reach)) {
      add_subtour_constraint(mdl, x, cycle, n, m, reach);
      mdl.optimize();
    }
  }

  if (mdl.get(GRB_IntAttr_SolCount) < 1) {
    unfix_vars(current, x); return;
  }

  current->LLB = max(current->LLB, mdl.get(GRB_DoubleAttr_ObjVal));

  pair<double,vector<triple>> heuristic_solution = upper_bound(current, in);
  double LUB = heuristic_solution.first;
  if (LUB < GUB) {
    GUB = LUB;
    copy_solution(sol, heuristic_solution.second);
  }

  if (current->LLB > GUB) { 
    unfix_vars(current, x); return;
  } 

  covering_constraints(x, mdl, n, m, in, reach, reached);

  int next_node = child(current, reach);
  if (next_node != -1) {
    current->lc = new Node(n, m, next_node, current, 1, current->LLB);
    current->rc = new Node(n, m, next_node, current, 0, current->LLB);
    col.push(node_ptr(current->lc));
    col.push(node_ptr(current->rc));
  } else if (mdl.get(GRB_DoubleAttr_ObjVal) < GUB) {
    GUB = mdl.get(GRB_DoubleAttr_ObjVal);
    update_solution(x, n, m, reach, sol);
  }
  unfix_vars(current, x);  
}

double branch_and_bound(GRBModel& mdl, x_vars& x, const int& n, const int& m, 
                        vector<triple>& sol, const vector<set<int>>& reach,
                        const vector<set<int>>& reached,
                        const Input& in) {

  double GUB = MAX_D;
  auto comparator = [](const node_ptr l, const node_ptr r) { return l->LLB > r->LLB; };
  priority_queue<node_ptr, vector<node_ptr>, decltype(comparator)> pq(comparator);
  pq.push(make_shared<Node>(n, m, -1, nullptr, -1, 0.0));

  while (!pq.empty() && pq.top()->LLB < GUB) {
    solve_and_branch(pq, mdl, x, n, m, reach, reached, GUB, sol, true, in);
  }

  return GUB;
}
