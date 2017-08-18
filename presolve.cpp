#include "presolve.h"

vector<set<int>> get_reach(const input& in, const int& n) { 
  vector<set<int>> reach;
  reach.emplace_back();

  for (int i = 1; i <= n+1; i++)
    reach[0].insert(i);

  for (int i = 1; i <= n; i++) {
    reach.emplace_back();
    pair<int,double> min[GAMMA];
    for (int j = 0; j < GAMMA; j++)
      min[j] = pair<int,double>(-1, MAX_D);

    for (int j = 1; j <= n; j++) {
      if (i == j)
        continue;

      int index = 0;
      while (index < GAMMA && in.d.at(int_pair(i,j)) > min[index].second)
        index++;

      if (index < GAMMA) {
        for (int b = GAMMA - 1; b > index; b--)
          min[b] = min[b-1];
        min[index] = pair<int,double>(j, in.d.at(int_pair(i,j)));
      }
    }

    for (pair<int,double> p : min)
      if (get<0>(p) > 0)
        reach[i].insert(get<0>(p));
    reach[i].insert(n+1);
  }

  reach.emplace_back();

  return reach;
}

vector<set<int>> get_reached(const vector<set<int>>& reach, const int& n) {
  vector<set<int>> reached;

  for (int i = 0; i <= n+1; i++)
    reached.emplace_back();

  for (int i = 0; i <= n+1; i++)
    for (int j : reach[i])
      reached[j].insert(i);

  return reached;
}

void increment_with_mst(vector<set<int>>& reach, vector<set<int>>& reached, const input& in, const int& n) {
  vector<tuple<int, int, double>> edges;

  for (int i = 1; i <= n; i++)
    for (int j = i+1; j <= n; j++)
      edges.push_back(make_tuple(i, j, in.d.at(int_pair(i, j)))); /* simetrico */

  sort(edges.begin(), edges.end(), 
      [](const tuple<int,int,double>& a, const tuple<int,int,double>& b) {
        return get<2>(a) < get<2>(b);
      });

  vector<tuple<int,int,double>> mst;
  UnionFind uf = UnionFind(n);

  for (tuple<int,int,double> edge : edges) {
    int i = get<0>(edge);
    int j = get<1>(edge);

    if (uf.find(i-1) == uf.find(j-1))
      continue;

    mst.push_back(edge);
    uf.join(i-1, j-1);
  }

  for (tuple<int,int,double> edge : mst) {
    int i = get<0>(edge);
    int j = get<1>(edge);

    reach[i].insert(j);
    reach[j].insert(i);
    reached[j].insert(i);
    reached[i].insert(j);
  }
}

