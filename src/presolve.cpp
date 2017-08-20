#include "presolve.h"

static set<int> closest_delivers(const int& i, const Input& in, const int& n) {
  vector<pair<int,double>> min = vector<pair<int,double>>(GAMMA);
  fill(min.begin(), min.end(), pair<int,double>(-1, MAX_D));

  for (int j = 1; j <= n; j++) {
    if (i == j)
      continue;

    int index = 0;
    while (index < GAMMA && in.d.at(int_pair(i,j)) > min[index].second)
      index++;

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
  closest.insert(n+1);

  return closest;
}

static vector<set<int>> closest_delivers(const Input& in, const int& n) { 
  vector<set<int>> reach;

  reach.emplace_back();
  for (int i = 1; i <= n+1; i++) {
    reach[0].insert(i);
  }

  for (int i = 1; i <= n; i++) {
    reach.emplace_back();

    for (int j : closest_delivers(i, in, n)) {
      reach[i].insert(j);
    }
  }

  reach.emplace_back();

  return reach;
}

static void increment_with_mst(vector<set<int>>& reach, const Input& in, const int& n) {
  vector<tuple<int, int, double>> edges;

  for (int i = 1; i <= n; i++)
    for (int j = i+1; j <= n; j++)
      edges.push_back(make_tuple(i, j, in.d.at(int_pair(i, j)))); /* simetrico */

  sort(edges.begin(), edges.end(), 
      [](const tuple<int,int,double>& a, const tuple<int,int,double>& b) {
        return get<2>(a) < get<2>(b);
      });

  vector<tuple<int,int,double>> mst;
  UnionFind uf = UnionFind(n+1);

  for (tuple<int,int,double> edge : edges) {
    int i = get<0>(edge);
    int j = get<1>(edge);

    if (uf.find(i) == uf.find(j))
      continue;

    mst.push_back(edge);
    uf.join(i, j);
  }

  for (tuple<int,int,double> edge : mst) {
    int i = get<0>(edge);
    int j = get<1>(edge);

    reach[i].insert(j);
    reach[j].insert(i);
  }
}

static vector<set<int>> reached_from_reach(const vector<set<int>>& reach, const int& n) {
  vector<set<int>> reached;

  for (int i = 0; i <= n+1; i++)
    reached.emplace_back();

  for (int i = 0; i <= n+1; i++)
    for (int j : reach[i])
      reached[j].insert(i);

  return reached;
}

pair<vector<set<int>>, vector<set<int>>> allowed_variables(const Input& in, const int& n) {
  vector<set<int>> reach = closest_delivers(in, n);
  increment_with_mst(reach, in, n);
  vector<set<int>> reached = reached_from_reach(reach, n);

  return pair<vector<set<int>>, vector<set<int>>>(reach, reached);
}

