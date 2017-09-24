#ifndef GRAPH_H
#define GRAPH_H
#include "ll_node.h" 
#include "uf.h"
#include "defs.h"

class Graph {
  private:
    int dfs_cycle(int v, vector<int>& pre, vector<int>& post, vector<int>& parent, int& precount, int& postcount);
  public:
    int n, m;
    vector<LinkedList> adj;
    vector<int> indeg, outdeg;
    Graph(int _n);
    Graph(int _n, const set<pair<int,int>>& edges);
    void add_arc(int from, int to);
    vector<int> tour(void);
    vector<Edge> all_edges(void);
    vector<Edge> mst(const function<bool (const Edge& a, const Edge& b)>& comparator);
};

#endif
