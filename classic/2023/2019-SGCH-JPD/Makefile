CXX       = g++
CXXFLAGS   = -W -O3 -std=c++11 -DNO_HOG -Wno-sign-compare -g
DEVFLAGS = -W -ggdb -O0 -std=c++11
EXEC     = run

SG_PATH = source
SG_INC = \
  -I$(SG_PATH) \
  -I$(SG_PATH)/CH_GPPC \
  -I$(SG_PATH)/ContractionHierarchy \
  -I$(SG_PATH)/Graphs \
  -I$(SG_PATH)/Graphs/Grid2D \
  -I$(SG_PATH)/Graphs/LatticeGraph \
  -I$(SG_PATH)/Graphs/LatticeGraph/Heuristic \
  -I$(SG_PATH)/Graphs/LatticeGraph/Lattice \
  -I$(SG_PATH)/Graphs/LatticeGraph/LatticeFreespace \
  -I$(SG_PATH)/Parameters \
  -I$(SG_PATH)/QueryMethods \
  -I$(SG_PATH)/ReachabilityRelation \
  -I$(SG_PATH)/ReachabilityRelation/Grid2D \
  -I$(SG_PATH)/ReachabilityRelation/Lattice \
  -I$(SG_PATH)/SearchMethods \
  -I$(SG_PATH)/SubgoalGraph \
  -I$(SG_PATH)/SubgoalGraph/Construction \
  -I$(SG_PATH)/SubgoalGraph/Grid2D \
  -I$(SG_PATH)/SubgoalGraph/Lattice \
  -I$(SG_PATH)/SubgoalGraph/Search \
  -I$(SG_PATH)/Utils \
  -I$(SG_PATH)/Utils/Experiments \
  -I$(SG_PATH)/Utils/Statistics

APP_SG = $(SG_PATH)
SRC_CPP = \
  $(APP_SG)/Graphs/Grid2D/Grid2D.cpp \
  $(APP_SG)/Graphs/Grid2D/GPPCGrid.cpp \
  $(APP_SG)/Graphs/LatticeGraph/Lattice/MotionPrimitive.cpp \
  $(APP_SG)/Graphs/LatticeGraph/Lattice/Lattice.cpp \
  $(APP_SG)/Graphs/LatticeGraph/LatticeGraph.cpp \
  $(APP_SG)/Graphs/LatticeGraph/LatticeFreespace/LatticeFreespace.cpp \
  $(APP_SG)/SubgoalGraph/Lattice/LatticeSG.cpp \
  $(APP_SG)/Graphs/CommonHeuristics.cpp \
  $(APP_SG)/ContractionHierarchy/ContractionCore.cpp \
  $(APP_SG)/Utils/CPUTimer.cpp \
  $(APP_SG)/CH_GPPC/mapper.cpp

all:
	$(CXX) $(CXXFLAGS) $(SG_INC) -o $(EXEC) *.cpp $(SRC_CPP)
dev:
	$(CXX) $(DEVFLAGS) $(SG_INC) -o $(EXEC) *.cpp $(SRC_CPP)
