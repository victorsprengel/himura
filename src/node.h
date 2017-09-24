#ifndef NODE_H
#define NODE_H
#include "defs.h"

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
    bool has_children(void);
    bool has_left_child(void);
    bool has_right_child(void);
};

using node_ptr = shared_ptr<Node>;

void spawn_children(shared_ptr<Node> leaf, const vector<set<int>>& reach);

#endif

