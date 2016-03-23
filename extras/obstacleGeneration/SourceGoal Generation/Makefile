CXX=g++
INC_DIR = ./
CXXFLAGS=-std=c++11 -I$(INC_DIR)
OBJ = sourceGoalGenerate.o

%.o: %.c
	$(CXX) -c -o $@ $< $(CXXFLAGS)

SGGenerator: $(OBJ)
	$(CXX) -o $@ $^ $(CXXFLAGS)
	rm -f *.o *~ core

.PHONY: clean

clean:
	rm -f *.o *~ core

