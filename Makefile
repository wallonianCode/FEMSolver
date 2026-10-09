CXX = g++
#CXXFLAGS = -std=c++20 -Wall -Wextra -I/usr/include/eigen3 
CXXFLAGS = -std=c++20 -Wall -Wextra -IC:\Users\z004ryjh\Desktop\GIT\FEA\Libraries\eigen-5.0.0\Eigen


TARGET = executable

SOURCES = src/main.cpp $(wildcard src/fem/*.cpp)
OBJECTS = $(SOURCES:.cpp=.o)

$(TARGET): $(OBJECTS)
				$(CXX) $(OBJECTS) -o $(TARGET)

%.o: %.cpp
				$(CXX) $(CXXFLAGS) -c $< -o $@


clean:
			  rm -f $(OBJECTS) $(TARGET) *.exe
