# ApproximationAlgorithm
An approximation algorithm i built using c++, more explained in the README file.

Approximate TSP Solver using MST (2-Approximation Algorithm)

Overview

This project implements a 2-Approximation Algorithm for the Traveling Salesman Problem (TSP) using a Minimum Spanning Tree (MST). Instead of solving TSP exactly (which is NP-hard), this algorithm provides a near-optimal tour in polynomial time.

Algorithm Explanation

The algorithm follows these steps:
Compute MST using Prim's Algorithm.
Perform a preorder DFS walk on the MST (Euler Tour).
Shortcut duplicate nodes to get a valid TSP tour.
The final tour is at most 2 times the optimal solution.

Complexity

MST (Prim’s Algorithm) -> O(E log V)
DFS Preorder Walk -> O(V + E)
Shortcut Duplicates -> O(V)
Total Complexity: O(E log V) (efficient for large graphs)


Installation:


Clone the Repository

git clone https://github.com/yourusername/tsp-approximation.git
cd tsp-approximation

Compile & Run

g++ Main.cpp -o tsp
./tsp

Example Output

- Approximate TSP Tour: 0 1 3 2 0  
- Approximate Cost: 10

Future Improvements

- Implement 1.5-Approximation (Christofides' Algorithm)
- Extend to 3D space with different distance metrics
- Optimize memory usage

Author: Scartissue

