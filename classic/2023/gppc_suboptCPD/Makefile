CXX       = g++
CXXFLAGS   = -W -Wall -O3 -std=c++17 -fopenmp
DEVFLAGS = -W -Wall -ggdb -O0 -std=c++17 -fopenmp
EXEC     = run

all:
	$(CXX) $(CXXFLAGS) -o $(EXEC) *.cpp
dev:
	$(CXX) $(DEVFLAGS) -o $(EXEC) *.cpp
