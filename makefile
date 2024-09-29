# Makefile

CXX = g++
CXXFLAGS = -std=c++20 -pthread -Wall -Wextra
OBJ_DIR = obj
SRC = active_object.cpp client.cpp graph.cpp kruskal_mst.cpp mst_factory.cpp prim_mst.cpp server.cpp
OBJ = $(SRC:%.cpp=$(OBJ_DIR)/%.o)

# Targets
all: server client

server: $(OBJ_DIR)/server.o $(OBJ_DIR)/active_object.o $(OBJ_DIR)/graph.o $(OBJ_DIR)/kruskal_mst.o $(OBJ_DIR)/mst_factory.o $(OBJ_DIR)/prim_mst.o
	$(CXX) $(CXXFLAGS) -o server $(OBJ_DIR)/server.o $(OBJ_DIR)/active_object.o $(OBJ_DIR)/graph.o $(OBJ_DIR)/kruskal_mst.o $(OBJ_DIR)/mst_factory.o $(OBJ_DIR)/prim_mst.o

client: $(OBJ_DIR)/client.o $(OBJ_DIR)/graph.o
	$(CXX) $(CXXFLAGS) -o client $(OBJ_DIR)/client.o $(OBJ_DIR)/graph.o

$(OBJ_DIR)/%.o: %.cpp
	mkdir -p $(OBJ_DIR)
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -rf $(OBJ_DIR)/*.o server client

.PHONY: all clean