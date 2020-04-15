SRCS = src/corner.cpp src/follow_the_gap.cpp src/lidar_data.cpp src/obstacle.cpp src/gap.cpp

CXXFLAGS = -Wall -O2 -std=c++11

libftg.a: $(SRCS:%.cpp=%.o)
	ar rvs $@ $(SRCS:%.cpp=%.o)

clean:
	-rm -f $(SRCS:%.cpp=%.o) libftg.a
