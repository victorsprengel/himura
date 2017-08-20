CXX=g++
CPPFLAGS=-g -O2 -std=c++14 -Wall -Wextra -Wfloat-equal -Wundef -Wshadow -Wunreachable-code -Wpedantic -Wold-style-cast -Wcast-align -Wmissing-include-dirs -Wredundant-decls -L./lib
LIBS=-lgurobi70 -lgurobi_c++

router: build/main.o build/bab.o build/dist.o build/graph.o build/heuristics.o build/input.o build/node.o build/presolve.o build/uf.o build/ub_helper.o
	$(CXX) $(CPPFLAGS) $^ -o bin/router $(LIBS)

build/main.o: src/main.cpp src/input.h src/presolve.h src/bab.h src/graph.h src/gurobi_c++.h
	$(CXX) $(CPPFLAGS) -c $< -o $@ $(LIBS)

build/bab.o: src/bab.cpp src/bab.h src/input.h src/node.h src/graph.h src/gurobi_c++.h src/heuristics.h
	$(CXX) $(CPPFLAGS) -c $< -o $@ $(LIBS)

build/dist.o: src/dist.cpp src/dist.h
	$(CXX) $(CPPFLAGS) -c $< -o $@ $(LIBS)

build/graph.o: src/graph.cpp src/graph.h
	$(CXX) $(CPPFLAGS) -c $< -o $@ $(LIBS)

build/heuristics.o: src/heuristics.cpp src/heuristics.h src/graph.h src/node.h src/gurobi_c++.h src/ub_helper.h src/input.h
	$(CXX) $(CPPFLAGS) -c $< -o $@ $(LIBS)

build/input.o: src/input.cpp src/input.h src/dist.h src/gurobi_c++.h
	$(CXX) $(CPPFLAGS) -c $< -o $@ $(LIBS)

build/node.o: src/node.cpp src/node.h 
	$(CXX) $(CPPFLAGS) -c $< -o $@ $(LIBS)

build/presolve.o: src/presolve.cpp src/presolve.h src/uf.h src/input.h
	$(CXX) $(CPPFLAGS) -c $< -o $@ $(LIBS)

build/uf.o: src/uf.cpp src/uf.h
	$(CXX) $(CPPFLAGS) -c $< -o $@ $(LIBS)

build/ub_helper.o: src/ub_helper.cpp src/ub_helper.h src/graph.h
	$(CXX) $(CPPFLAGS) -c $< -o $@ $(LIBS)

clean:
	rm -f build/* bin/*

test: router
	./bin/router vehicles.data packages.data

testout: router
	./bin/router vehicles.data packages.data > out
