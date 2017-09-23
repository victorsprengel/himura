#ifndef GRAPH_H
#define GRAPH_H
#include <vector>
#include <stack>
#include <algorithm>
#include "ll_node.h" 
using namespace std;
using int_pair = pair<int,int>;
using LinkedList = shared_ptr<LinkedListNode>;

class Graph {
  private:
    int dfs_cycle(int v, vector<int>& pre, vector<int>& post, vector<int>& parent, int& precount, int& postcount);
  public:
    int n, m;
    vector<LinkedList> adj;
    vector<int> indeg, outdeg;
    Graph(int _n);
    void add_arc(int from, int to);
    vector<int> tour(void);
};

#endif
