#include "input.h"

static vector<string> split(string s, char c) {
  vector<string> r;
  size_t i;
  while (s.size() > 0) {
    i = s.find(c);
    r.push_back(s.substr(0, i));

    if (i == string::npos) {
      s.clear();
    } else {
      s = s.substr(i+1);
    }
  }
  return r;
}

void input::read(char *inVehicles, char *inDeliveries, int& n, int& m) {
  m = 0;
  ifstream inV(inVehicles);
  string line;

  while (getline(inV, line)) {
    vector<string> s_line = split(line, ' ');
    int qt = stoi(s_line[0]);
    m += qt;
    for (int i = 0; i < qt; i++) {
      C.push_back(stod(s_line[1]));
      V.push_back(stod(s_line[2]));
      M.push_back(stod(s_line[3]));
      H.push_back(stod(s_line[4]));
      F.push_back(stod(s_line[5]));
      E.push_back(stod(s_line[6]));
      S.push_back(stod(s_line[7]));
      T.push_back(stod(s_line[8]));
      J.push_back(stod(s_line[9]));
    }
  }

  n = -1; /* primeiro é o CO */
  ifstream inD(inDeliveries);
  vector<float> lat, lon;

  while (getline(inD, line)) {
    if (line[0] == '#')
      continue;
    n++;
    vector<string> sp = split(line, ',');
    lat.push_back(stod(sp[0]));
    lon.push_back(stod(sp[1]));
    p.push_back(stod(sp[2]));
    v.push_back(stod(sp[3]));
  }

  cout << "n = " << n << "    m = " << m << endl;
  double max_dist = 0.0;

  for (int i = 0; i <= n; i++) {
    for (int j = 1; j <= n + 1; j++) {
      if (i >= j)
        continue;
      if (j == n + 1)
        d[pii(i,j)] = 0;
      else {
        d[pii(i,j)] = distance(lat[i], lon[i], lat[j], lon[j]);
        if (d[pii(i,j)] > max_dist)
          max_dist = d[pii(i,j)];
        if (i > 0)
          d[pii(j,i)] = d[pii(i,j)];
      }
    }
  }
  cout << "maximum distance between two points: " << max_dist << "km" << endl;
}

input::input() {

}
