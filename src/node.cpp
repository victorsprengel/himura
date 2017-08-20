#include "node.h"

static int corresponding_var(const int& i, const int& j, const int& k, const int& n) {
  return k * (n+1) * (n+1) + (j-1) * (n+1) + i;
}

int child(shared_ptr<Node> leaf, const vector<set<int>>& reach) {
  int n = leaf->n;
  int m = leaf->m;
  vector<bool> visited = vector<bool>(n+2);
  fill(visited.begin(), visited.end(), false);
  vector<deque<int>> path = vector<deque<int>>(m);
  vector<vector<vector<bool>>> fixed = vector<vector<vector<bool>>>(n+2);
  for (int i = 0; i <= n; i++) {
    fixed[i] = vector<vector<bool>>(n+2);
    for (int j = 1; j <= n+1; j++) {
      fixed[i][j] = vector<bool>(m);
      for (int k = 0; k < m; k++) {
        fixed[i][j][k] = false;
      }
    }
  }

  while (leaf->parent != nullptr) {
    int i = leaf->i(), j = leaf->j(), k = leaf->k();
    fixed[i][j][k] = true;
    if (leaf->fixed_value) {
      path[k].push_front(j);
      visited[j] = true;
    }
    leaf = leaf->parent;
  }

  for (int k = 0; k < m; k++) {
    vector<int> options;
    int i = path[k].empty() ? 0 : path[k].back();
    for (int j : reach[i]) {
      if (fixed[i][j][k])
        continue;
      options.push_back(j);
    }
    if (!options.empty()) {
      stable_sort(options.begin(), options.end(), 
          [&](const int& a, const int& b) -> bool {return !visited[a] || visited[b];});

      return corresponding_var(i, options[0], k, n);
    }
  }

  return -1;
}

Node::Node(int _n, int _m, int _var, shared_ptr<Node> _parent, int _fixed, double _llb) {
  n = _n;
  m = _m;
  var = _var;
  parent = _parent;
  lc = nullptr;
  rc = nullptr;
  fixed_value = _fixed;
  LLB = _llb;
}

int Node::i() {
  return var % (n+1);
}

int Node::j() {
  return ((var / (n+1)) % (n+1)) + 1;
}

int Node::k() {
  return (var / (n+1)) / (n+1);
}

