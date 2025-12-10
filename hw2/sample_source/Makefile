# Makefile for Rasterizer Project

CXX = g++
CXXFLAGS = -std=c++11 -g -Wall
TARGET = rasterizer

# Source files
SOURCES = Main.cpp Camera.cpp Color.cpp Helpers.cpp Instance.cpp \
          Matrix4.cpp Mesh.cpp Rotation.cpp Scaling.cpp Scene.cpp \
          Translation.cpp Triangle.cpp Vec3.cpp Vec3WithColor.cpp \
          Vec4.cpp Vec4WithColor.cpp tinyxml2.cpp

# Object files
OBJECTS = $(SOURCES:.cpp=.o)

# Default target
all: $(TARGET)

# Build the executable
rasterizer: $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o rasterizer $(OBJECTS)

$(TARGET): $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJECTS)

# Compile source files to object files
%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

# Clean build artifacts
clean:
	rm -f $(OBJECTS) $(TARGET)

# Rebuild everything
rebuild: clean all

# Run the program with input file
run: $(TARGET)
	./$(TARGET) $(INPUT_FILE)

.PHONY: all clean rebuild run
