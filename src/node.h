#ifndef NODE_H
#define NODE_H
#include <set>
#include <vector>
#include <cstddef>
#include <memory>
using namespace std;

class Node {
  public:
    int n, m, depth, times_called, fixed_value;
    double LLB;
    Node *lc, *rc;
    shared_ptr<Node> parent;
    Node(int _n, int _m, int d, shared_ptr<Node> _parent, int _fixed, double g, const vector<set<int>>& reach);
    int i();
    int j();
    int k();
    bool has_children(const vector<set<int>>& reach);
};

#endif

