#include "heuristics.h"

static bool contains(const vector<pair<int,int>>& v, const pair<int,int>& q) {
  return find(v.begin(), v.end(), q) != v.end();
}

static bool violates_c(const int& k, const input& in, const ub_info& ubh) {
  return !(ubh.usedVol[k] <= in.C[k] && ubh.usedVal[k] <= in.V[k] && ubh.usedTime[k] <= in.J[k]);
}

static bool would_violate_c(const int& i, const int& j, const int& k, const input& in, const ub_info &ubh) {
  return !((ubh.usedVol[k] + in.v[j]) <= in.C[k] &&
           (ubh.usedVal[k] + in.p[j]) <= in.V[k] &&
           (ubh.usedTime[k] + ((in.d.at(int_pair(i,j)) / in.M[k]) + in.T[k])) <= in.J[k]);
}

static void add_package(const int& i, const int& j, const int& k, const input& in, ub_info& ubh) {
  ubh.graph[k].add_arc(i, j);
  ubh.usedVal[k] += in.p[j];
  ubh.usedVol[k] += in.v[j];
  ubh.usedTime[k] += ((in.d.at(int_pair(i,j)) / in.M[k]) + in.T[k]);
  ubh.in[j] = true;
  ubh.out[i] = true;
}

static bool fill_graphs_with_fixed(node_ptr leaf, const input& in, ub_info& ubh, const int& n) {
  node_ptr current = leaf, parent;

  while ((parent = current->parent) != nullptr) {
    int i = parent->i(), j = parent->j(), k = parent->k();

    if (current->fixed_value) {
      if ((ubh.out[i] && i != 0) || (ubh.in[j] && j != n+1))
        return false;

      add_package(i, j, k, in, ubh);

      if (violates_c(k, in, ubh))
        return false;
    } else {
      ubh.blocked[k].push_back(int_pair(i, j));
    }
    current = parent;
  }
  return true;
}

static vector<int> get_sources(const Graph& g, const int& n) {
  vector<int> srcs;
  for (int i = 1; i <= n; i++)
    if (!g.indeg[i] && g.outdeg[i])
      srcs.push_back(i);
  return srcs;
}

static int best_next_source(vector<int>& srcs, const int& from, const input& in, const ub_info& ubh, const int& k) {
  double min_dist = MAX_D;
  int w_index = -1;

  for (size_t i = 0; i < srcs.size(); i++) {
    bool closer = in.d.at(int_pair(from,srcs[i])) < min_dist;
    bool not_blocked = !contains(ubh.blocked[k], int_pair(from,srcs[i]));
    bool fits = !would_violate_c(from, srcs[i], k, in, ubh);

    if (closer && not_blocked && fits) {
      w_index = i;
      min_dist = in.d.at(int_pair(from,srcs[i]));
    }
  }

  return w_index;
}

static bool make_graphs_paths(ub_info& ubh, const input& in, const int& n, const int& m) {
  for (int k = 0; k < m; k++) {
    vector<int> srcs = get_sources(ubh.graph[k], n);
    int v = 0;
    while (!(srcs.empty())) {
      v = ubh.graph[k].sink(v);
      if (v == n+1)
        return false;

      int w_index = best_next_source(srcs, v, in, ubh, k);
      if (w_index == -1)
        return false;

      int w = srcs[w_index];
      srcs.erase(srcs.begin() + w_index);
      add_package(v, w, k, in, ubh);
    }
  }
  return true;
}

static int best_vehicle(const int& deliver, ub_info& ubh, const input& in, const int& n, const int& m) {
  double min_dist = MAX_D;
  int chosen_k = -1;

  for (int k = 0; k < m; k++) {
    int previous_stop = ubh.graph[k].sink(0);
    if (previous_stop == n+1)
      continue;

    bool closer = in.d.at(int_pair(previous_stop,deliver)) < min_dist;
    bool not_blocked = !contains(ubh.blocked[k], int_pair(previous_stop,deliver));
    bool fits = !would_violate_c(previous_stop, deliver, k, in, ubh);
    if (closer && not_blocked && fits) {
      min_dist = in.d.at(int_pair(previous_stop, deliver));
      chosen_k = k;
    }
  }

  return chosen_k;
}

static bool deliver_remaining(ub_info& ubh, const input& in, const int& n, const int& m) {
  for (int j = 1; j <= n; j++) {
    if (!ubh.in[j]) {
      int k = best_vehicle(j, ubh, in, n, m);
      if (k == -1)
        return false;

      /* TODO sink sendo feito 2x */
      add_package(ubh.graph[k].sink(0), j, k, in, ubh);
    }
  }
  return true;
}

static void go_to_sink(ub_info& ubh, const int& n, const int& m) {
  for (int k = 0; k < m; k++) {
    int last = ubh.graph[k].sink(0);
    if (last != n+1)
      ubh.graph[k].add_arc(last, n+1);
  }
}

static vector<triple> graphs_to_solution(ub_info& ubh, const int& m) {
  vector<triple> sol;
  for (int k = 0; k < m; k++)
    for (int_pair arc : ubh.graph[k].all_arcs())
      sol.push_back(make_tuple(arc.first, arc.second, k));
  return sol;
}

static double solution_to_value(const vector<triple>& sol, const input& in, const int& n, const int& m) {
  double total = 0.0;
  vector<bool> used = vector<bool>(m);
  fill(used.begin(), used.end(), false);

  for (triple t : sol) {
    int i = get<0>(t);
    int j = get<1>(t);
    int k = get<2>(t);

    if (j == n+1)
      continue;

    used[k] = true;
    total += ((in.H[k] * in.T[k]) + 
              (in.H[k] * (in.d.at(int_pair(i,j)) / in.M[k])) + 
              (in.E[k]) + 
              (in.F[k] * in.d.at(int_pair(i,j))));
  }
  
  for (int k = 0; k < m; k++)
    if (used[k])
      total += in.S[k];

  return total;
}

pair<double,vector<triple>> upper_bound(node_ptr current, const input& in) {
  int n = current->n, m = current->m;
  ub_info ubh = ub_info(n, m);

  if (!fill_graphs_with_fixed(current, in, ubh, n)) {
    return pair<double,vector<triple>>(MAX_D, vector<triple>());
  }

  if (!make_graphs_paths(ubh, in, n, m)) {
    return pair<double,vector<triple>>(MAX_D, vector<triple>());
  }

  if (!deliver_remaining(ubh, in, n , m)) {
    return pair<double,vector<triple>>(MAX_D, vector<triple>());
  }

  go_to_sink(ubh, n, m);
  vector<triple> solution = graphs_to_solution(ubh, m);
  double total = solution_to_value(solution, in, n, m);

  return pair<double,vector<triple>>(total, solution);
}

vector<int> subtour_elimination_heuristic(x_vars& x, const int& n, const int& m, const vector<set<int>>& reach) {
  Graph g = Graph(n+2);
  for (int i = 0; i <= n; i++) {
    for (int j : reach[i]) {
      double sum = 0.0;
      for (int k = 0; k < m; k++)
        sum += x[i][j][k].get(GRB_DoubleAttr_X);
      if (sum > ALPHA)
        g.add_arc(i, j);
    }
  }
  vector<int> cycle = g.tour();
  return cycle;
}

void covering_constraints(x_vars& x, GRBModel& model, const int& n, const int& m, const input& in, const vector<set<int>>& reach, const vector<set<int>>& reached) {
  for (int k = 0; k < m; k++) {
    set<int> S;

    for (int i = 0; i <= n; i++) {
      for (int j : reach[i]) {
        if (j == n+1)
          continue;
        if (x[i][j][k].get(GRB_DoubleAttr_X) > BETA)
          S.insert(j);
      }
    }

    double used_vol = 0.0, used_val = 0.0;
    for (int j : S) {
      used_val += in.p[j];
      used_vol += in.v[j];
    }

    if (used_val > in.V[k] || used_vol > in.C[k]) {
      GRBLinExpr e = 0.0;
      for (int j : S) {
        for (int i : reached[j]) {
          e += x[i][j][k];
        }
      }
      model.addConstr(e, GRB_LESS_EQUAL, S.size() - 1);
    }
  }
}

