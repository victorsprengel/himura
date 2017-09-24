#include "presolve.h"

static set<int> closest_delivers(const int& i, const Input& in) {
  vector<pair<int,double>> min = vector<pair<int,double>>(GAMMA);
  fill(min.begin(), min.end(), pair<int,double>(-1, MAX_D));

  for (int j = 1; j <= in.n; j++) {
    if (i == j)
      continue;

    int index = 0;
    while (index < GAMMA && in.d.at(int_pair(i,j)) > min[index].second) {
      index++;
    }

    if (index < GAMMA) {
      for (int b = GAMMA - 1; b > index; b--) {
        min[b] = min[b-1];
      }
      min[index] = pair<int,double>(j, in.d.at(int_pair(i,j)));
    }
  }

  set<int> closest;
  for (pair<int,double> p : min) {
    if (p.first > 0) {
      closest.insert(p.first);
    }
  }

  return closest;
}

static vector<Partition> closest_delivers(const Input& in) { 
  vector<Partition> reach;

  reach.emplace_back();
  for (int i = 1; i <= in.n + 1; i++) {
    reach[0].insert(i);
  }

  for (int i = 1; i <= in.n; i++) {
    reach.emplace_back();

    for (int j : closest_delivers(i, in)) {
      reach[i].insert(j);
    }
    reach[i].insert(in.n + 1);
  }

  reach.emplace_back();

  return reach;
}

vector<Partition> find_reach(const Input& in) {
  vector<Partition> reach = closest_delivers(in);

  Graph G = Graph(in.n + 1);
  for (int i = 1; i <= in.n; i++) {
    for (int j = i+1; j <= in.n; j++) {
      G.add_arc(i, j);
    }
  }

  vector<Edge> mst = G.mst([&](const Edge& a, const Edge& b) 
      { return in.d.at(int_pair(a.first, a.second)) < in.d.at(int_pair(b.first, b.second)); });

  for (Edge e : mst) {
    reach[e.first].insert(e.second);
  }

  return reach;
}

vector<Partition> reached_from_reach(const vector<Partition>& reach, const int& n) {
  vector<Partition> reached = vector<Partition>(n+2);

  for (size_t i = 0; i < reach.size(); i++) {
    for (int j : reach[i]) {
      reached[j].insert(i);
    }
  }
  
  return reached;
}
