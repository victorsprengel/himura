#include "input.h"

static vector<string> split(string s, char c) {
  vector<string> r;
  size_t i;
  while (s.size() > 0) {
    i = s.find(c);
    string entry = s.substr(0, i);

    if (entry.size()) {
      r.push_back(entry);
    }

    if (i == string::npos) {
      s.clear();
    } else {
      s = s.substr(i+1);
    }
  }
  return r;
}

Input::Input(char *inVehicles, char *inDeliveries) {
  m = 0;
  ifstream inV(inVehicles);
  string line;

  while (getline(inV, line)) {
    vector<string> sp = split(line, ' ');
    int qt = stoi(sp[0]);
    m += qt;
    for (int i = 0; i < qt; i++) {
      C.push_back(stod(sp[1]));
      V.push_back(stod(sp[2]));
      M.push_back(stod(sp[3]));
      H.push_back(stod(sp[4]));
      F.push_back(stod(sp[5]));
      E.push_back(stod(sp[6]));
      S.push_back(stod(sp[7]));
      T.push_back(stod(sp[8]));
      J.push_back(stod(sp[9]));
    }
  }

  n = -1; /* 0 is depot */
  ifstream inD(inDeliveries);

  while (getline(inD, line)) {
    if (line[0] == '#') {
      continue;
    }

    n++;
    vector<string> sp = split(line, ',');
    lat.push_back(stod(sp[0]));
    lon.push_back(stod(sp[1]));
    p.push_back(stod(sp[2]));
    v.push_back(stod(sp[3]));
  }

  for (int i = 0; i <= n; i++) {
    for (int j = i+1; j <= n + 1; j++) {

      if (j == n + 1) {
        d[pair<int,int>(i,j)] = 0;
      } else {
        d[pair<int,int>(i,j)] = distance(lat[i], lon[i], lat[j], lon[j]);

        if (i > 0) {
          d[pair<int,int>(j,i)] = d[pair<int,int>(i,j)];
        }
      }
    }
  }
}

