#ifndef LL_NODE_H
#define LL_NODE_H
#include "defs.h"

class LinkedListNode {
  public:
    int v;
    shared_ptr<LinkedListNode> next;
    void insert(int new_v);
};

using LinkedList = shared_ptr<LinkedListNode>;
using LLNode = shared_ptr<LinkedListNode>;

#endif
