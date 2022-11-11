CXX = g++
CXXFLAGS = -Wall -g -std=c++11 -fopenmp
LIBS = -pthread -lm -fopenmp

main: main.o evolve.o setPoints.o createObject.o
	$(CXX) main.o evolve.o setPoints.o createObject.o $(LIBS) -o main

main.o: main.cpp evolve.hpp setPoints.hpp createObject.hpp utility.h

evolve.o: evolve.cpp evolve.hpp setPoints.hpp createObject.hpp utility.h

setPoints.o: setPoints.cpp setPoints.hpp createObject.hpp utility.h

createObject.o: createObject.cpp createObject.hpp utility.h

.PHONY: clean
clean:
	rm -f *.o main

.PHONY: all
all: clean make
