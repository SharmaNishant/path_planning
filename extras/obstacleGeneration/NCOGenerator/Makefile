CXX=g++
INC_DIR = ./
CXXFLAGS=-std=c++11 -I$(INC_DIR)
CXXFLAGS += `pkg-config --cflags opencv` `pkg-config --libs opencv`
LIBS = `pkg-config --libs opencv`
OBJ = NCOGenerator.o

#%.o: %.c $(DEPS)
%.o: %.c
	$(CXX) -c -o $@ $< $(CXXFLAGS)

NCOGenerator: $(OBJ)
	$(CXX) -o $@ $^ $(CXXFLAGS)
	rm -f *.o *~ core

.PHONY: clean

clean:
	rm -f *.o *~ core

