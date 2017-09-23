#include "graph.h"

Graph::Graph(int _n) {
  n = _n;
  m = 0;
  for (int i = 0; i < n; i++) {
    indeg.push_back(0);
    outdeg.push_back(0);
    adj.push_back(nullptr);
  }
}

void Graph::add_arc(int from, int to) {
  LinkedListNode* new_node = new LinkedListNode();
  new_node->v = to;
  new_node->next = adj[from];
  adj[from] = shared_ptr<LinkedListNode>(new_node);
}

vector<int> Graph::tour(void) {
  vector<int> pre, post, parent, c;
  int possibleCycle, precount = 0, postcount = 0;

  for (int i = 0; i < n; i++) {
    pre.push_back(-1);
    post.push_back(-1);
    parent.push_back(-1);
  }

  for (int v = 0; v < Graph::n; v++) {
    if (pre[v] == -1) {
      parent[v] = v;
      if ((possibleCycle = Graph::dfs_cycle(v, pre, post, parent, precount, postcount)) != -1) {
        int start = possibleCycle;
        do {
          c.push_back(possibleCycle);
          possibleCycle = parent[possibleCycle];
        } while (possibleCycle != start);
        reverse(c.begin(), c.end());
        return c;
      }
    }
  }
  return c;
}

int Graph::dfs_cycle(int v, vector<int>& pre, vector<int>& post, vector<int>& parent, int& precount, int& postcount) {
  pre[v] = precount++;
  vector<int> eds;

  shared_ptr<LinkedListNode> current = adj[v];
  while (current != nullptr) {
    eds.push_back(current->v);
    current = current->next;
  }

  random_shuffle(eds.begin(), eds.end());

  for (int w : eds) {
    if (pre[w] == -1) {
      parent[w] = v;
      int possibleCycle = Graph::dfs_cycle(w, pre, post, parent, precount, postcount);
      if (possibleCycle != -1)
        return possibleCycle;
    } else if (pre[v] > pre[w] && post[w] == -1) {
      parent[w] = v;
      return v;
    }
  }
  post[v] = postcount++;
  return -1;
}

