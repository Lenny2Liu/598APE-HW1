CXX       := g++
CXXFLAGS  := -O3 -g -Werror -fopenmp -std=c++11
LDFLAGS   := -lm -fopenmp

OBJ_DIR   := bin

CPP_FILES         := $(wildcard src/*.cpp)
OBJ_FILES         := $(patsubst src/%.cpp,$(OBJ_DIR)/%.o,$(CPP_FILES))
TEX_CPP_FILES     := $(wildcard src/Textures/*.cpp)
TEX_OBJ_FILES     := $(patsubst src/Textures/%.cpp,$(OBJ_DIR)/Textures/%.o,$(TEX_CPP_FILES))

TARGET    := main.exe

all: $(TARGET)

$(TARGET): main.cpp $(OBJ_FILES) $(TEX_OBJ_FILES)
	$(CXX) $(CXXFLAGS) main.cpp $(OBJ_FILES) $(TEX_OBJ_FILES) -o $@ $(LDFLAGS)

$(OBJ_DIR)/%.o: src/%.cpp
	mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(OBJ_DIR)/Textures/%.o: src/Textures/%.cpp
	mkdir -p $(OBJ_DIR)/Textures
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJ_DIR)/*.o $(OBJ_DIR)/Textures/*.o $(TARGET)
