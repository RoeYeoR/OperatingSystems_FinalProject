# Minimum Spanning Tree (MST) Client-Server Application

## Project Overview

This project implements a client-server application for calculating and analyzing Minimum Spanning Trees (MST) in graphs. The system demonstrates several advanced operating systems concepts including:

- **Client-Server Architecture**: Communication between client and server processes
- **Concurrency Patterns**: 
  - Leader-Follower Thread Pool
  - Active Object Pattern with Pipeline Processing
- **MST Algorithms**:
  - Kruskal's Algorithm
  - Prim's Algorithm
- **Graph Analysis**:
  - Total MST Weight
  - Longest Distance between Vertices
  - Average Distance between Vertices
  - Shortest Path between Selected Vertices

## Features

- **Graph Input**: Client can define a graph by specifying vertices and edges with weights
- **Algorithm Selection**: Choose between Prim's or Kruskal's algorithm for MST calculation
- **Real-time Algorithm Switching**: Change MST algorithm during runtime
- **Path Analysis**: Calculate shortest paths between any two vertices in the graph
- **Concurrent Processing**: Server handles multiple client requests simultaneously using thread pools
- **Pipeline Processing**: Three-stage pipeline for reading, processing, and sending data

## System Architecture

### Components

1. **Server**: 
   - Accepts client connections
   - Processes graph data using selected MST algorithm
   - Calculates metrics (total weight, distances)
   - Sends results back to clients

2. **Client**:
   - Connects to server
   - Sends graph data and algorithm choice
   - Receives and displays MST metrics
   - Allows for algorithm switching and path calculations

3. **MST Implementation**:
   - Strategy Pattern for algorithm selection
   - Factory Pattern for creating MST algorithm instances
   - Common implementation for shared functionality

4. **Concurrency Model**:
   - Leader-Follower Thread Pool for connection handling
   - Active Pipeline for staged processing of client requests

## Building and Running

### Prerequisites

- C++ compiler with C++20 support
- POSIX-compliant operating system (Linux/macOS/WSL)
- Make build system

### Compilation

To build both the server and client applications:

```bash
make all
```

To build only the server:

```bash
make server
```

To build only the client:

```bash
make client
```

To clean the build files:

```bash
make clean
```

### Running the Application

1. **Start the Server**:
   ```bash
   ./server
   ```

2. **Start the Client**:
   ```bash
   ./client
   ```

3. **Using the Client**:
   - Enter the number of vertices
   - Enter the number of edges
   - For each edge, enter source vertex, destination vertex, and weight
   - Choose MST algorithm (Prim/Kruskal)
   - View the MST metrics (total weight, longest distance, average distance)
   - Choose options:
     1. Change MST Algorithm
     2. Calculate shortest distance between two vertices
     3. Exit

## Implementation Details

### MST Algorithms

- **Kruskal's Algorithm**: Uses Disjoint Set data structure to find MST
- **Prim's Algorithm**: Uses priority queue to find MST

### Concurrency Patterns

- **Leader-Follower Thread Pool**: 
  - One thread (leader) accepts connections
  - Remaining threads (followers) process client requests
  - Dynamic promotion of followers to leader role

- **Active Pipeline**:
  - Three-stage pipeline: Read → Process → Send
  - Each stage runs in its own thread
  - Tasks flow through the pipeline asynchronously

## Memory Management

The project uses modern C++ memory management techniques:
- Smart pointers for automatic resource management
- RAII principles for resource acquisition and release
- Exception-safe design

## Error Handling

- Robust error handling for socket operations
- Graceful handling of client disconnections
- Proper cleanup of resources on shutdown

## Performance Considerations

- Thread pool size adapts to hardware concurrency
- Efficient graph representation for MST algorithms
- Optimized path finding algorithms

## Example Usage

1. Start the server
2. Connect with the client
3. Define a graph (e.g., 4 vertices, 5 edges)
4. Choose Kruskal's algorithm
5. View the MST metrics
6. Calculate shortest path between vertices 0 and 3
7. Switch to Prim's algorithm and compare results

## License

This project is an educational demonstration of operating systems concepts and concurrency patterns.
