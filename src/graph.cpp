#include "graph.h"

Graph::Graph(int _n) {
  n = _n;
  m = 0;
  for (int i = 0; i < n; i++) {
    first.push_back(-1);
    indeg.push_back(0);
    outdeg.push_back(0);
  }
}

void Graph::add_arc(int from, int to) {
  indeg[to]++;
  outdeg[from]++;
  arcs.push_back(to);
  next.push_back(first[from]);
  first[from] = m++;
}

void Graph::add_edge(const pair<int,int>& edge) {
  add_arc(edge.first, edge.second);
  add_arc(edge.second, edge.first);
}

int Graph::sink(int v) {
  while (first[v] != -1) {
    v = arcs[first[v]];
  }
  return v;
}

vector<int> Graph::tour(void) {
  vector<int> pre, post, parent, c;
  int possibleCycle, precount = 0, postcount = 0;

  for (int i = 0; i < n; i++) {
    pre.push_back(-1);
    post.push_back(-1);
    parent.push_back(-1);
  }

  for (int v = 0; v < Graph::n; v++) {
    if (pre[v] == -1) {
      parent[v] = v;
      if ((possibleCycle = Graph::dfs_cycle(v, pre, post, parent, precount, postcount)) != -1) {
        int start = possibleCycle;
        do {
          c.push_back(possibleCycle);
          possibleCycle = parent[possibleCycle];
        } while (possibleCycle != start);
        reverse(c.begin(), c.end());
        return c;
      }
    }
  }
  return c;
}

int Graph::dfs_cycle(int v, vector<int>& pre, vector<int>& post, vector<int>& parent, int& precount, int& postcount) {
  pre[v] = precount++;
  vector<int> eds;

  for (int e = first[v]; e != -1; e = next[e]) {
    eds.push_back(arcs[e]);
  }
  random_shuffle(eds.begin(), eds.end());

  for (int w : eds) {
    if (pre[w] == -1) {
      parent[w] = v;
      int possibleCycle = Graph::dfs_cycle(w, pre, post, parent, precount, postcount);
      if (possibleCycle != -1)
        return possibleCycle;
    } else if (pre[v] > pre[w] && post[w] == -1) {
      parent[w] = v;
      return v;
    }
  }
  post[v] = postcount++;
  return -1;
}

vector<int_pair> Graph::all_arcs() {
  vector<int_pair> all;
  for (int i = 0; i < n; i++)
    for (int e = first[i]; e != -1; e = next[e])
      all.push_back(int_pair(i,arcs[e]));
  return all;
}


