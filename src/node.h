#ifndef NODE_H
#define NODE_H
#include <set>
#include <vector>
#include <algorithm>
#include <cstddef>
#include <memory>
#include <deque>
using namespace std;

class Node {
  public:
    int n, m, var, fixed_value;
    double LLB;
    Node *lc, *rc;
    shared_ptr<Node> parent;
    Node(int _n, int _m, int _var, shared_ptr<Node> _parent, int _fixed, double _llb);
    int i();
    int j();
    int k();
};

int child(shared_ptr<Node> leaf, const vector<set<int>>& reach);

#endif

