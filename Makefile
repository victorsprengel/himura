CXX=g++
CPPFLAGS=-g -O2 -std=c++14 -Wall -Wextra -Wfloat-equal -Wundef -Wshadow -Wunreachable-code -Wpedantic -Wold-style-cast -Wcast-align -Wmissing-include-dirs -Wredundant-decls -L./lib
LIBS=-lgurobi70 -lgurobi_c++

main.o: main.cpp input.h presolve.h bab.h graph.h
	$(CXX) $(CPPFLAGS) -c main.cpp

input.o: input.cpp dist.h input.h
	$(CXX) $(CPPFLAGS) -c input.cpp

dist.o: dist.cpp dist.h
	$(CXX) $(CPPFLAGS) -c dist.cpp

presolve.o: presolve.cpp input.h presolve.h uf.h
	$(CXX) $(CPPFLAGS) -c presolve.cpp

bab.o: bab.cpp bab.h input.h node.h graph.h heuristics.h
	$(CXX) $(CPPFLAGS) -c bab.cpp

node.o: node.cpp node.h
	$(CXX) $(CPPFLAGS) -c node.cpp

graph.o: graph.cpp graph.h
	$(CXX) $(CPPFLAGS) -c graph.cpp

heuristics.o: heuristics.h graph.h heuristics.cpp ub_helper.h node.h input.h
	$(CXX) $(CPPFLAGS) -c heuristics.cpp

ub_helper.o: ub_helper.cpp ub_helper.h graph.h
	$(CXX) $(CPPFLAGS) -c ub_helper.cpp

uf.o: uf.h uf.cpp
	$(CXX) $(CPPFLAGS) -c uf.cpp

router: main.o input.o dist.o presolve.o bab.o node.o graph.o heuristics.o ub_helper.o uf.o
	$(CXX) $(CPPFLAGS) main.o input.o dist.o presolve.o bab.o node.o graph.o heuristics.o ub_helper.o uf.o -o router $(LIBS)

clean:
	rm -f router *.o

