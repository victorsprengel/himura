#include "uf.h"

UnionFind::UnionFind(const int& n) {
  parent = vector<int>(n);
  tree_size = vector<int>(n);
  for (int i = 0; i < n; i++) {
    parent[i] = i;
    tree_size[i] = 1;
  }
}

int UnionFind::find(const int& a) {
  if (parent[a] == a)
    return a;
  return (parent[a] = find(parent[a]));
}

void UnionFind::join(const int& a, const int& b) {
  int x = find(a);
  int y = find(b);
  if (tree_size[y] > tree_size[x]) {
    int temp = y;
    y = x;
    x = temp;
  }
  parent[y] = x;
  tree_size[x] += tree_size[y];
}
    
