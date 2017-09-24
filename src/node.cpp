#include "node.h"

static int corresponding_var(
    const int& i, 
    const int& j, 
    const int& k, 
    const int& n) {

  return k * (n+1) * (n+1) + (j-1) * (n+1) + i;
}

void spawn_children(
    shared_ptr<Node> leaf, 
    const vector<set<int>>& reach) {

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

  node_ptr current = leaf;
  while (current->parent != nullptr) {
    int i = current->i(), j = current->j(), k = current->k();
    fixed[i][j][k] = true;
    if (current->fixed_value) {
      path[k].push_front(j);
      visited[j] = true;
    }
    current = current->parent;
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

      int next_node = corresponding_var(i, options[0], k, n);
      leaf->lc = new Node(n, m, next_node, leaf, 1, leaf->LLB);
      leaf->rc = new Node(n, m, next_node, leaf, 0, leaf->LLB);
      return;
    }
  }

  for (int k = 0; k < m; k++) {
    for (int i = 0; i <= n; i++) {
      for (int j : reach[i]) {
        if (!fixed[i][j][k]) {
          int next_node = corresponding_var(i, j, k, n);
          leaf->rc = new Node(n, m, next_node, leaf, 0, leaf->LLB);
          return;
        }
      }
    }
  }
}

Node::Node(
    int _n, 
    int _m, 
    int _var, 
    shared_ptr<Node> _parent, 
    int _fixed, 
    double _llb) {

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

bool Node::has_children(
    void) {

  return has_left_child() || has_right_child();
}

bool Node::has_left_child(
    void) {

  return lc != nullptr;
}

bool Node::has_right_child(
    void) {

  return rc != nullptr;
}


