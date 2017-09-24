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

Graph::Graph(int _n, const set<pair<int,int>>& edges) {
  n = _n;
  m = 0;
  for (int i = 0; i < n; i++) {
    indeg.push_back(0);
    outdeg.push_back(0);
    adj.push_back(nullptr);
  }
  for (pair<int,int> edge : edges) {
    add_arc(edge.first, edge.second);
    add_arc(edge.second, edge.first);
  }
}

void Graph::add_arc(int from, int to) {
  LinkedListNode* new_node = new LinkedListNode();
  new_node->v = to;
  new_node->next = adj[from];
  adj[from] = shared_ptr<LinkedListNode>(new_node);
}

vector<int> Graph::dicircuit(void) {
  vector<int> pre = vector<int>(n);
  vector<int> post = vector<int>(n);
  vector<int> parent = vector<int>(n);
  vector<int> c;
  
  fill(pre.begin(), pre.end(), -1);
  fill(post.begin(), post.end(), -1);
  fill(parent.begin(), parent.end(), -1);
  int possibleCycle, precount = 0, postcount = 0;

  for (int v = 0; v < n; v++) {
    if (pre[v] == -1) {
      parent[v] = v;
      if ((possibleCycle = dfs_circuit(v, pre, post, parent, precount, postcount)) != -1) {
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

int Graph::dfs_circuit(
    int v, 
    vector<int>& pre, 
    vector<int>& post, 
    vector<int>& parent, 
    int& precount, 
    int& postcount) {

  pre[v] = precount++;
  vector<int> eds;

  LLNode current = adj[v];
  while (current != nullptr) {
    eds.push_back(current->v);
    current = current->next;
  }

  random_shuffle(eds.begin(), eds.end());

  for (int w : eds) {
    if (pre[w] == -1) {
      parent[w] = v;
      int possibleCycle = Graph::dfs_circuit(w, pre, post, parent, precount, postcount);
      if (possibleCycle != -1) {
        return possibleCycle;
      }
    } else if (pre[v] > pre[w] && post[w] == -1) {
      parent[w] = v;
      return v;
    }
  }
  post[v] = postcount++;
  return -1;
}

vector<Edge> Graph::all_edges(void) {
  vector<Edge> edges;

  for (int i = 0; i < n; i++) {
    LLNode current = adj[i];
    while (current != nullptr) {
      if (current->v > i) {
        edges.push_back(Edge(i, current->v));
      }
      current = current->next;
    }
  }

  return edges;
}

vector<Edge> Graph::mst(
    const function<bool (const Edge& a, const Edge& b)>& comparator) {

  vector<Edge> edges = all_edges(), mst_edges;

  sort(edges.begin(), edges.end(), comparator);

  UnionFind uf = UnionFind(n);

  for (Edge e : edges) {
    int i = e.first;
    int j = e.second;

    if (uf.find(i) == uf.find(j))
      continue;

    mst_edges.push_back(e);
    uf.join(i, j);
  }

  return mst_edges;
}
