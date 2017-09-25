#include "ub.h"

static n3_int state_of_variables(
    const int& n, 
    const int& m, 
    const List<int_triple>& used,
    const List<int_triple>& blocked,
    const vector<Partition>& reach) {

  n3_int states = n3_int(n + 1);
  for (Delivery i = 0; i <= n; i++) {
    states[i] = n2_int(n + 2);
    for (Delivery j : reach[i]) {
      states[i][j] = n1_int(m);
      for (Vehicle k = 0; k < m; k++) {
        states[i][j][k] = FREE;
      }
    }
  }

  for (int_triple t : used) {
    states[get<0>(t)][get<1>(t)][get<2>(t)] = USED;
  }

  for (int_triple t : blocked) {
    states[get<0>(t)][get<1>(t)][get<2>(t)] = BLOCKED;
  }

  return states;
}


static List<Partition> kruskal_like_partitioning(
    const List<int_triple>& used,
    const Input& in) {

  List<Partition> assignments;
  vector<int> owner = vector<int>(in.n + 2);
  vector<double> usedVal = vector<double>(in.m), usedVol = vector<double>(in.m);

  fill(owner.begin(), owner.end(), -1);
  fill(usedVol.begin(), usedVol.end(), 0.0);
  fill(usedVal.begin(), usedVal.end(), 0.0);

  for (Vehicle k = 0; k < in.m; k++) {
    assignments.push_back(Partition());
    assignments[k].insert(0);
  }

  for (int_triple fixed_var : used) {
    int i,j,k;
    tie(i,j,k) = fixed_var;

    if ((i != 0 && owner[i] != -1 && owner[i] != k) ||
        (j != in.n + 1 && owner[j] != -1 && owner[j] != k) ||
        (i != 0 && owner[i] == -1 && (usedVal[k] + in.p[i] > in.V[k] || usedVol[k] + in.v[i] > in.C[k])) ||
        (j != in.n + 1 && owner[j] == -1 && (usedVal[k] + in.p[j] > in.V[k] || usedVol[k] + in.v[j] > in.C[k]))) {
      throw FIXED_VARS_GIVE_SAME_PACKAGE_TO_TWO_DIFFERENT_VEHICLES;
    }

    if (i != 0 && owner[i] == -1) {
      usedVal[k] += in.p[i];
      usedVol[k] += in.v[i];
      owner[i] = k;
    }

    assignments[k].insert(i);

    if (j != in.n + 1) {
      if (owner[j] == -1) {
        usedVal[k] += in.p[j];
        usedVol[k] += in.v[j];
        owner[j] = k;
      }

      assignments[k].insert(j);
    }
  }

  List<Delivery> missing_deliveries;
  for (Delivery i = 1; i <= in.n; i++) { 
    if (owner[i] == -1) {
      missing_deliveries.push_back(i);
    }
  }

  for (Delivery j : missing_deliveries) {
    Vehicle best_vehicle = -1;
    double best_distance = MAX_D;

    for (Vehicle k = 0; k < in.m; k++) {
      for (Delivery i : assignments[k]) {
        double dist = in.d.at(int_pair(i, j));

        if (dist < best_distance && usedVol[k] + in.v[j] <= in.C[k] && usedVal[k] + in.p[j] <= in.V[k]) {
          best_vehicle = k;
          best_distance = dist;
        }
      }
    }

    if (best_vehicle == -1) {
      throw NO_PARTITION_CAN_FIT_ANOTHER_PACKAGE;
    }

    assignments[best_vehicle].insert(j);
    usedVol[best_vehicle] += in.v[j];
    usedVal[best_vehicle] += in.p[j];
    owner[j] = best_vehicle;
  }

  return assignments;
  
}

static List<Edge> mst_of_partition(
    const Partition& partition,
    const Vehicle& k,
    const int& n,
    const n3_int& var_status,
    const map<Edge,double>& cost,
    const vector<Partition>& reach) {

  Graph G = Graph(n+1);

  for (Delivery i : partition) {
    for (Delivery j : partition) {
      if (j <= i) {
        continue;
      }

      G.add_arc(i, j);
    }
  }

  return G.mst([&](const Edge& a, const Edge& b)
      { double cost_a = cost.at(a), cost_b = cost.at(b);
        
        if (reach[a.first].find(a.second) != reach[a.first].end()) {
          if (var_status[a.first][a.second][k] == BLOCKED) {
            cost_a = MAX_D;
          } else if (var_status[a.first][a.second][k] == USED) {
            cost_a = 0.0;
          }
        }

        if (reach[b.first].find(b.second) != reach[b.first].end()) {
          if (var_status[b.first][b.second][k] == BLOCKED) {
            cost_b = MAX_D;
          } else if (var_status[b.first][b.second][k] == USED) {
            cost_b = 0.0;
          }
        }

        return cost_a < cost_b;
      });
}

static set<Delivery> odd_degree_vertices(
    const List<Edge>& mst, 
    const int& n) {

  set<Delivery> O;
  vector<Delivery> deg = vector<int>(n+1);
  fill(deg.begin(), deg.end(), 0);

  for (Edge e : mst) {
    deg[e.first]++;
    deg[e.second]++;
  }

  for (Delivery i = 0; i <= n; i++) {
    if (deg[i] % 2 == 1) {
      O.insert(i);
    }
  }

  return O;
}

static List<Edge> greedy_min_weighted_perfect_matching (
    set<Delivery>& odds, 
    const map<Edge,double>& cost,
    const Vehicle& k,
    const n3_int& var_status,
    const vector<Partition>& reach) {

  List<Edge> matching;
  
  while (!odds.empty()) {
    double best_edge_cost = MAX_D;
    Edge best_edge;

    for (int u : odds) {
      for (int v : odds) {
        if (v <= u) {
          continue;
        }

        Edge e = Edge(u, v);
        double edge_cost = cost.at(e);

        if (reach[u].find(v) != reach[u].end()) {
          if (var_status[u][v][k] == BLOCKED) {
            edge_cost = MAX_D;
          } else if (var_status[u][v][k] == USED) {
            edge_cost = 0.0;
          }
        } 

        if (abs(MAX_D - best_edge_cost) <= 1e-8 || edge_cost < best_edge_cost) {
          best_edge_cost = edge_cost;
          best_edge = e;
        }
      }
    }

    odds.erase(best_edge.first);
    odds.erase(best_edge.second);
    matching.push_back(best_edge);
  }

  return matching;
}

static Dicircuit euler(Graph& G) {
  Dicircuit circuit;
  stack<Delivery> stk;
  Delivery current = 0;

  while (!stk.empty() || G.adj[current] != nullptr) {
    if (G.adj[current] == nullptr) {
      circuit.push_back(current);
      current = stk.top();
      stk.pop();
    } else {
      stk.push(current);
      Delivery neighbour = G.adj[current]->v;
      G.adj[current] = G.adj[current]->next;
      current = neighbour;
    }
  }

  return circuit;
}

static Dicircuit hamiltonian_from_eulerian(const Dicircuit& eulerian_circuit, const int& n) {
  Dicircuit hamiltonian_circuit;
  vector<bool> visited = vector<bool>(n+1);
  fill(visited.begin(), visited.end(), false);

  for (Delivery d : eulerian_circuit) {
    if (!visited[d]) {
      hamiltonian_circuit.push_back(d);
      visited[d] = true;
    }
  }

  return hamiltonian_circuit;
}

static void two_opt(Dicircuit& circuit, const Input& in) {
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

static List<Delivery> christofides(
    const Partition& partition,
    const Vehicle& k,
    const List<int_triple>& used, 
    const List<int_triple>& blocked, 
    const vector<Partition>& reach,
    const Input& in) {

  List<Delivery> route;

  if (partition.size() <= 1) {
    return List<Delivery>();
  }

  n3_int var_status = state_of_variables(in.n, in.m, used, blocked, reach);

  List<Edge> mst = mst_of_partition(partition, k, in.n, var_status, in.d, reach);
  set<Delivery> O = odd_degree_vertices(mst, in.n);
  List<Edge> matching = greedy_min_weighted_perfect_matching(O, in.d, k, var_status, reach);

  List<Edge> multigraph;

  for (Edge e : mst) {
    multigraph.push_back(e);
  }

  for (Edge e : matching) {
    multigraph.push_back(e);
  }

  Graph G = Graph(in.n + 1, multigraph);
  Dicircuit eulerian = euler(G);
  Dicircuit hamiltonian = hamiltonian_from_eulerian(eulerian, in.n);
  two_opt(hamiltonian, in);

  return hamiltonian;
}

static bool is_viable_route(
    const Dicircuit& route,
    const Input& in,
    const Vehicle& k) {

  double usedTime = 0.0;

  for (size_t l = 0; l < route.size() -1; l++) {
    usedTime += (in.T[k] + in.d.at(Edge(route[l], route[l+1])) / in.M[k]);
  }

  return usedTime <= in.J[k];
}

Solution heuristic_solution(
    const List<int_triple>& used, 
    const List<int_triple>& blocked, 
    const vector<Partition>& reach,
    const Input& in) {

  List<int_triple> solution;
  List<Partition> assignments = kruskal_like_partitioning(used, in);

  for (Vehicle k = 0; k < in.m; k++) {
    List<Delivery> tsp_solution = christofides(assignments[k], k, used, blocked, reach, in);

    if (!tsp_solution.size()) {
      continue;
    }

    if (!is_viable_route(tsp_solution, in, k)) {
      throw FOUND_TOUR_DOESNT_RESPECT_WORKING_HOURS;
    }

    tsp_solution.push_back(in.n + 1);

    for (size_t i = 0; i < tsp_solution.size() - 1; i++) {
      int u = tsp_solution[i];
      int v = tsp_solution[i+1];

      solution.push_back(make_tuple(u,v,k));
    }
  }

  return solution;
}
