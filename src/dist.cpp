#include "dist.h"

static double deg2rad(double deg) {
  return deg * (pi / 180.0);
}

/* haversine formula - rosetta code*/
double distance(double lat1, double lon1, double lat2, double lon2) {
  double d_lat = deg2rad(lat2 - lat1);
  double d_lon = deg2rad(lon2 - lon1);
  lat1 = deg2rad(lat1);
  lat2 = deg2rad(lat2);
  double a = pow(sin(d_lat / 2), 2) + cos(lat1) * cos(lat2) * pow(sin(d_lon / 2), 2);
  double b = 2 * asin(sqrt(a));
  return R * b;
}

