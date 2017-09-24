CXX=g++
CPPFLAGS=-g -O2 -std=c++14 -Wall -Wextra -Wfloat-equal -Wundef -Wshadow -Wunreachable-code -Wpedantic -Wold-style-cast -Wcast-align -Wmissing-include-dirs -Wredundant-decls -fdiagnostics-color=always -L./lib 
LIBS=-lgurobi70 -lgurobi_c++

bin/router: build/main.o build/input.o build/presolve.o build/dist.o build/dist.o build/graph.o build/ll_node.o build/uf.o build/gurobi_interface.o build/bab.o build/heuristics.o build/debug.o build/node.o
	$(CXX) $(CPPFLAGS) $^ -o bin/router $(LIBS)

build/main.o: src/main.cpp src/defs.h src/input.h src/presolve.h src/gurobi_interface.h src/debug.h
	$(CXX) $(CPPFLAGS) -c $< -o $@ $(LIBS)

build/input.o: src/input.cpp src/input.h src/dist.h
	$(CXX) $(CPPFLAGS) -c $< -o $@ $(LIBS)

build/presolve.o: src/presolve.cpp src/presolve.h src/input.h src/graph.h src/defs.h
	$(CXX) $(CPPFLAGS) -c $< -o $@ $(LIBS)

build/dist.o: src/dist.cpp src/dist.h src/defs.h
	$(CXX) $(CPPFLAGS) -c $< -o $@ $(LIBS)

build/graph.o: src/graph.cpp src/graph.h src/defs.h src/ll_node.h src/uf.h
	$(CXX) $(CPPFLAGS) -c $< -o $@ $(LIBS)

build/ll_node.o: src/ll_node.cpp src/ll_node.h src/defs.h
	$(CXX) $(CPPFLAGS) -c $< -o $@ $(LIBS)

build/uf.o: src/uf.cpp src/uf.h src/defs.h
	$(CXX) $(CPPFLAGS) -c $< -o $@ $(LIBS)

build/gurobi_interface.o: src/gurobi_interface.cpp src/gurobi_interface.h src/defs.h
	$(CXX) $(CPPFLAGS) -c $< -o $@ $(LIBS)

build/bab.o: src/bab.cpp src/bab.h src/defs.h src/node.h src/heuristics.h src/graph.h
	$(CXX) $(CPPFLAGS) -c $< -o $@ $(LIBS)

build/heuristics.o: src/heuristics.cpp src/heuristics.h src/graph.h src/input.h src/defs.h
	$(CXX) $(CPPFLAGS) -c $< -o $@ $(LIBS)

build/node.o: src/node.cpp src/node.h src/defs.h
	$(CXX) $(CPPFLAGS) -c $< -o $@ $(LIBS)

build/debug.o: src/debug.cpp src/debug.h src/defs.h src/input.h src/graph.h
	$(CXX) $(CPPFLAGS) -c $< -o $@ $(LIBS)

clean:
	rm -f build/* bin/*

test: bin/router
	./bin/router vehicles.data packages.data

testout: bin/router
	./bin/router vehicles.data packages.data > out
