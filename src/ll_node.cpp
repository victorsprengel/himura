#include "ll_node.h"

void LinkedListNode::insert(int new_v) {
  shared_ptr<LinkedListNode> tmp = shared_ptr<LinkedListNode>(next);
  LinkedListNode* new_node = new LinkedListNode();
  new_node->v = new_v;
  new_node->next = shared_ptr<LinkedListNode>(tmp);
  next = shared_ptr<LinkedListNode>(new_node);
}
