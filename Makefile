CXX := g++
CXXFLAGS := -std=c++11 -fPIC -g -Wall -O3
TARGET := main
SRCS := $(notdir $(wildcard *.cpp))
OBJS := $(patsubst %.cpp, %.o, $(SRCS))

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $<

clean:
	rm -f $(TARGET) $(OBJS)