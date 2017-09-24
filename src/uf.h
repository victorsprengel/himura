#ifndef UF_H_
#define UF_H_
#include "defs.h"

class UnionFind {
  private:
    vector<int> parent;
    vector<int> tree_size;
  public:
    UnionFind(const int& n);
    void join(const int& a, const int& b);
    int find(const int& a);
};

#endif
