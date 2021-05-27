
INCLUDES=$(wildcard *.hpp)
SRCS=$(wildcard *.cpp)
CXXFLAGS=-DUSE_GRAPHICS # -DSF2K_EFFICIENCY
CXX=g++ -g -std=c++17 -O3
MAIN=main
LIBS=-lsfml-window -lsfml-system -lsfml-graphics

all: $(MAIN)

%.o: %.cpp $(INCLUDES) Makefile

$(MAIN): $(SRCS:.cpp=.o)
	$(CXX) $(LDFLAGS) $^ -o $@ $(LIBS)

clean:
	rm -f $(MAIN) $(SRCS:.cpp=.o)


