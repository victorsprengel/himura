#ifndef LL_NODE_H
#define LL_NODE_H
#include <memory>
using namespace std;
using int_pair = pair<int,int>;

class LinkedListNode {
  public:
    int v;
    shared_ptr<LinkedListNode> next;
    void insert(int new_v);
};

#endif
