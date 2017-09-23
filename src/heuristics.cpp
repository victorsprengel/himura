#include "heuristics.h"

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
  return g.tour();
}

void covering_constraints(x_vars& x, GRBModel& model, const int& n, const int& m, const Input& in, const vector<set<int>>& reach, const vector<set<int>>& reached) {
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

static vector<set<int>> next_neighbour(node_ptr& current, const Input& in) {
  int n = current->n, m = current->m;

  vector<set<int>> partition;

  vector<int> vehicles;
  for (int k = 0; k < m; k++) {
    vehicles.push_back(k);
    partition.push_back(set<int>());
  }

  sort(vehicles.begin(), vehicles.end(),
      [&](const int& a, const int& b) -> bool {return in.S[a] < in.S[b];});

  vector<bool> visited = vector<bool>(n+1);
  fill(visited.begin(), visited.end(), false);

  for (int k : vehicles) {
    int i = 0;
    visited[i] = true;
    partition[k].insert(i);
    double used_val = 0.0, used_vol = 0.0, used_time = 0.0;

    for ( ; ; ) {
      int next_deliver = -1;
      double min_dist = MAX_D;

      for (int j = 1; j <= n; j++) {
        if (!visited[j] && 
            min_dist > in.d.at(pair<int,int>(i,j)) &&
            used_val + in.p[j] <= in.V[k] &&
            used_vol + in.v[j] <= in.C[k] &&
            used_time + in.T[k] + (in.d.at(pair<int,int>(i,j)) / in.M[k]) <= in.J[k]) {

          next_deliver = j;
          min_dist = in.d.at(pair<int,int>(i,j));
        }
      }

      if (next_deliver == -1) {
        break;
      }

      used_val += in.p[next_deliver];
      used_vol += in.v[next_deliver];
      used_time += (in.T[k] + (in.d.at(pair<int,int>(i,next_deliver)) / in.M[k]));
      visited[next_deliver] = true;
      i = next_deliver;
      partition[k].insert(next_deliver);
    }
  }

  for (int i = 1; i <= n; i++) {
    if (!visited[i]) {
      throw 1;
    }
  }

  return partition;
}

static set<pair<int, int>> mst_of_partition(const set<int>& partition, const int& n, const Input& in) {
  vector<tuple<int,int,double>> edges;
  set<pair<int,int>> mst;

  for (int i : partition) {
    for (int j : partition) {
      if (j <= i) {
        continue;
      }

      edges.push_back(make_tuple(i, j, in.d.at(int_pair(i, j)))); /* simetrico */
    }
  }

  sort(edges.begin(), edges.end(), 
      [](const tuple<int,int,double>& a, const tuple<int,int,double>& b) {
      return get<2>(a) < get<2>(b);
      });

  UnionFind uf = UnionFind(n+1);

  for (tuple<int,int,double> edge : edges) {
    int i = get<0>(edge);
    int j = get<1>(edge);

    if (uf.find(i) == uf.find(j)) {
      continue;
    }

    mst.insert(pair<int,int>(i, j));
    uf.join(i, j);
  }

  return mst;
}

static set<int> odd_degree_vertices(const set<pair<int,int>>& mst, const int& n) {
  set<int> O;
  vector<int> deg = vector<int>(n+1);
  fill(deg.begin(), deg.end(), 0);

  for (pair<int,int> edge : mst) {
    deg[edge.first]++;
    deg[edge.second]++;
  }

  for (int i = 0; i <= n; i++) {
    if (deg[i] % 2 == 1) {
      O.insert(i);
    }
  }

  return O;
}

static set<pair<int,int>> greedy_min_weighted_perfect_matching(set<int>& odds, const Input& in) {
  set<pair<int,int>> matching;
  
  while (!odds.empty()) {
    int v = *(odds.begin());
    odds.erase(v);
    double length = MAX_D;
    int closest;

    for (int u : odds) {
      if (in.d.at(pair<int,int>(v, u)) < length) {
        length = in.d.at(pair<int,int>(v, u));
        closest = u;
      }
    }
    
    matching.insert(pair<int,int>(closest, v));
    odds.erase(closest);
  }

  return matching;
}

static vector<int> euler(Graph& G) {
  vector<int> circuit;
  stack<int> stk;
  int current = 0;

  while (!stk.empty() || G.adj[current] != nullptr) {
    if (G.adj[current] == nullptr) {
      circuit.push_back(current);
      current = stk.top();
      stk.pop();
    } else {
      stk.push(current);
      int neighbour = G.adj[current]->v;
      G.adj[current] = G.adj[current]->next;
      current = neighbour;
    }
  }

  return circuit;
}

static vector<int> hamilton(const vector<int>& eulerian_circuit, const int& n) {
  vector<int> hamiltonian_circuit;
  vector<bool> visited = vector<bool>(n+1);
  fill(visited.begin(), visited.end(), false);

  for (int vertex : eulerian_circuit) {
    if (!visited[vertex]) {
      hamiltonian_circuit.push_back(vertex);
      visited[vertex] = true;
    }
  }

  return hamiltonian_circuit;
}

static void two_opt(vector<int>& circuit, const Input& in) {
  bool changed;

  do {
    changed = false;

    for (size_t a = 0; a < circuit.size(); a++) {
      for (size_t c = a+1; c < circuit.size(); c++) {
        int b = a + 1 == circuit.size() ? 0 : a + 1;
        int d = c + 1 == circuit.size() ? 0 : c + 1;

        if (in.d.at(pair<int,int>(min(circuit[a],circuit[b]), max(circuit[a], circuit[b]))) + 
            in.d.at(pair<int,int>(min(circuit[c],circuit[d]), max(circuit[c], circuit[d]))) > 
            in.d.at(pair<int,int>(min(circuit[a],circuit[c]), max(circuit[a], circuit[c]))) +
            in.d.at(pair<int,int>(min(circuit[b],circuit[d]), max(circuit[b], circuit[d])))) { 

          int tmp = circuit[b];
          circuit[b] = circuit[c];
          circuit[c] = tmp;
          changed = true;
        }
      }
    }
  } while (changed);
}

static pair<double, vector<triple>> christofides(vector<set<int>> partitions, node_ptr& current, const Input& in) {
  int n = current->n, m = current->m;
  double total = 0;
  vector<triple> sol;

  for (int k = 0; k < m; k++) {
    if (partitions[k].size() <= 1) {
      continue;
    }

    total += in.S[k];
  
    set<pair<int,int>> mst = mst_of_partition(partitions[k], n, in); 
    set<int> O = odd_degree_vertices(mst, n);
    set<pair<int,int>> matching = greedy_min_weighted_perfect_matching(O, in);
    for (pair<int,int> edge : matching) {
      mst.insert(edge);
    }

    Graph G = Graph(n+1, mst);

    vector<int> eulerian_circuit = euler(G);

    vector<int> hamiltonian_circuit = hamilton(eulerian_circuit, n);
    
    two_opt(hamiltonian_circuit, in);
    hamiltonian_circuit.push_back(n+1);

    for (size_t i = 0; i < hamiltonian_circuit.size() - 1; i++) {
      int u = hamiltonian_circuit[i];
      int v = hamiltonian_circuit[i+1];

      sol.push_back(make_tuple(u,v,k));

      if (v == n+1) {
        continue;
      }
      
      total += (in.E[k] + in.F[k] * in.d.at(pair<int,int>(u,v)) + in.H[k] * (in.T[k] + in.d.at(pair<int,int>(u,v)) / in.M[k]));
    }
  }

  assert_viable_solution(sol, n, m, in, total);
  return pair<double, vector<triple>>(total, sol);
}

pair<double, vector<triple>> upper_bound(node_ptr& current, const Input& in) {
  try {
    vector<set<int>> partitions = next_neighbour(current, in);
    return christofides(partitions, current, in);
  } catch (int exception) {
    return pair<double,vector<triple>>(MAX_D, vector<triple>());
  }
}

