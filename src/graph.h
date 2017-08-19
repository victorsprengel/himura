#ifndef GRAPH_H
#define GRAPH_H
#include <vector>
#include <stack>
#include <algorithm>
using namespace std;
using int_pair = pair<int,int>;

class Graph {
  vector<int> next, arcs, first;
  int dfs_cycle(int v, vector<int>& pre, vector<int>& post, vector<int>& parent, int& precount, int& postcount);
 public:
   int n, m;
   vector<int> indeg, outdeg;
   Graph(int _n);
   void add_arc(int from, int to);
   vector<int> tour(void);
   int sink(int v);
   vector<int_pair> all_arcs(); 
};

#endif
