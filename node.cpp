#include "node.h"

Node::Node(int _n, int _m, int d, shared_ptr<Node> _parent, int _fixed, double l, const vector<set<int>>& reach) {
  times_called = 0;
  n = _n;
  m = _m;
  depth = d;
  while (reach[i()].find(j()) == reach[i()].end()) {
    depth++;
  }
  parent = _parent;
  lc = nullptr;
  rc = nullptr;
  fixed_value = _fixed;
  LLB = l;
}

int Node::i() {
  return depth % (n+1);
}

int Node::j() {
  return ((depth / (n+1)) % (n+1)) + 1;
}

int Node::k() {
  return (depth / (n+1)) / (n+1);
}

bool Node::has_children(const vector<set<int>>& reach) {
  int next_depth = depth;
  int ii, jj;
  do {
    next_depth++;
    ii = next_depth % (n+1);
    jj = ((next_depth / (n+1)) % (n+1)) + 1;
  } while (next_depth < (n+1)*(n+1)*m && reach[ii].find(jj) == reach[ii].end());

  return next_depth < (n+1)*(n+1)*m;
}

