# Himura

## What is it?

Himura is an efficient implementation of a Branch&Bound algorithm to solve one specific instance of the SCVRP.

## The instance

One company has to deliver daily to its clients. All vehicles start from a depot and don't have to return. 

The objective is to assign those delivers to vehicles and also sequence them in order to minimize costs. 

For the lack of data, we are using the haversine distance between two deliveries. Therefore this is a symmetric
case study of the problem.

Now to the singularities of each vehicle and deliver:

### The deliveries
- Volume (m³)
- Value ($)
- Address (lat,lon)

### The vehicles
- Storage capacity (m³)
- Value capacity ($)
- Average speed (km/h)
- Cost per hour ($/h)
- Cost per km ($/km)
- Fixed cost per deliver ($/deliver)
- Fixed daily cost ($)
- Average time to make a deliver (h)
- Total time available to deliver (h)

## It runs on...

C++14 and the GUROBI library.

