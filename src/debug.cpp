#include "debug.h"

void assert_viable_solution(
    const Solution& sol, 
    const Input& in, 
    const double& expected_val) {

  int n = in.n, m = in.m;

  for (int j = 1; j <= n; j++) {
    int delivered = 0;
    for (int_triple t : sol)
      if (get<1>(t) == j)
        delivered++;
    assert(delivered == 1);
  }

  for (int k = 0; k < m; k++) {
    double vol = 0.0;
    for (int_triple t : sol)
      if (get<2>(t) == k && get<1>(t) != n+1)
        vol += in.v[get<1>(t)];
    assert(vol <= in.C[k]);
  }


  for (int k = 0; k < m; k++) {
    double val = 0.0;
    for (int_triple t : sol)
      if (get<2>(t) == k && get<1>(t) != n+1)
        val += in.p[get<1>(t)];
    assert(val <= in.V[k]);
  }

  for (int k = 0; k < m; k++) {
    for (int j = 1; j <= n; j++) {
      int got_in = 0, got_out = 0;
      for (int_triple t : sol) {
        if (get<2>(t) == k) {
          if (get<0>(t) == j) {
            got_out++;
          }
          if (get<1>(t) == j) {
            got_in++;
          }
        }
      }
      assert(got_in == got_out && got_in <= 1 && got_out <= 1);
    }
  }

  for (int k = 0; k < m; k++) {
    double time = 0.0;
    for (int_triple t : sol) {
      if (get<2>(t) == k && get<1>(t) != n+1) {
        time += (in.T[k] + (in.d.at(int_pair(get<0>(t),get<1>(t))) / in.M[k]));
      }
    }
    assert(time <= in.J[k]);
  }

  for (int k = 0; k < m; k++) {
    Graph g = Graph(n+2);
    for (int_triple t : sol)
      if (get<2>(t) == k)
        g.add_arc(get<0>(t), get<1>(t));
    vector<int> cycle = g.tour();
    assert (cycle.size() == 0);
  }

  double solution = 0.0;
  for (int k = 0; k < m; k++) {
    bool used = false;
    for (int_triple t : sol)
      if (get<2>(t) == k && get<1>(t) != n+1)
        used = true;
    if (used)
      solution += in.S[k];
  }
  for (int_triple t : sol) {
    int i = get<0>(t);
    int j = get<1>(t);
    int k = get<2>(t);
    assert (i != j);
    assert (j != 0);
    assert (i != n+1);
    if (j == n+1)
      continue;
    solution += (in.H[k] * in.T[k]);
    solution += ((in.H[k] * in.d.at(int_pair(i,j))) / in.M[k]);
    solution += (in.E[k]);
    solution += (in.F[k] * in.d.at(int_pair(i,j)));
  }

  assert(abs(solution - expected_val) <= 1e-8);
}
