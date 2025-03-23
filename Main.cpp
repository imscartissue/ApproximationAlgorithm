#include <iostream>
#include <vector>
#include <queue>
#include <unordered_set>
#include <limits>

struct Edge {
	int to;
	double weight;
	Edge(int t, double w) : to(t), weight(w) {}
};

// Graph Representation (Adjacency List)
class Graph {
public:
	int V;
	std::vector<std::vector<Edge>> adj;

	Graph(int vertices) : V(vertices), adj(vertices) {}

	void addEdge(int u, int v, double weight) {
		adj[u].push_back(Edge(v, weight));
		adj[v].push_back(Edge(u, weight));  // Undirected Graph
	}

	// Prim's Algorithm to Compute MST
	std::vector<std::vector<int>> getMST() {
		std::vector<bool> inMST(V, false);
		std::vector<int> parent(V, -1);
		std::vector<double> key(V, std::numeric_limits<double>::max());

		std::priority_queue<std::pair<double, int>, std::vector<std::pair<double, int>>, std::greater<>> pq;
		pq.push({ 0, 0 });  // Start from node 0
		key[0] = 0;

		while (!pq.empty()) {
			int u = pq.top().second;
			pq.pop();

			if (inMST[u]) continue;
			inMST[u] = true;

			// Process neighbors
			for (auto& edge : adj[u]) {
				int v = edge.to;
				double weight = edge.weight;

				if (!inMST[v] && weight < key[v]) {
					key[v] = weight;
					parent[v] = u;
					pq.push({ key[v], v });
				}
			}
		}

		// Build MST as an adjacency list
		std::vector<std::vector<int>> mst(V);
		for (int i = 1; i < V; i++) {
			if (parent[i] != -1) {
				mst[parent[i]].push_back(i);
				mst[i].push_back(parent[i]);
			}
		}
		return mst;
	}
};

// Depth-First Search (DFS) Preorder Walk
void dfs(int node, const std::vector<std::vector<int>>& mst, std::unordered_set<int>& visited, std::vector<int>& tour) {
	visited.insert(node);
	tour.push_back(node);
	for (int neighbor : mst[node]) {
		if (!visited.count(neighbor)) {
			dfs(neighbor, mst, visited, tour);
			tour.push_back(node);  // Euler Tour (backtracking step)
		}
	}
}

// Approximate TSP Solver using MST + Preorder Walk
std::vector<int> tspApproximation(Graph& graph) {
	std::vector<std::vector<int>> mst = graph.getMST();

	// Step 2: Preorder DFS traversal of MST
	std::vector<int> eulerTour;
	std::unordered_set<int> visited;
	dfs(0, mst, visited, eulerTour);

	// Step 3: Shortcut repeated nodes to form Hamiltonian cycle
	std::vector<int> tspTour;
	std::unordered_set<int> seen;
	for (int node : eulerTour) {
		if (!seen.count(node)) {
			tspTour.push_back(node);
			seen.insert(node);
		}
	}
	tspTour.push_back(tspTour[0]);  // Complete the cycle

	return tspTour;
}

// Compute the total cost of the TSP Tour
double computeTourCost(const std::vector<int>& tour, Graph& graph) {
	double cost = 0.0;
	for (size_t i = 0; i < tour.size() - 1; i++) {
		int u = tour[i], v = tour[i + 1];
		for (auto& edge : graph.adj[u]) {
			if (edge.to == v) {
				cost += edge.weight;
				break;
			}
		}
	}
	return cost;
}

// Driver Code
int main() {
	Graph graph(4);

	// Adding edges (undirected)
	graph.addEdge(0, 1, 1);
	graph.addEdge(0, 2, 3);
	graph.addEdge(1, 2, 4);
	graph.addEdge(1, 3, 2);
	graph.addEdge(2, 3, 5);

	// Run Approximation Algorithm
	std::vector<int> tspTour = tspApproximation(graph);
	double approxCost = computeTourCost(tspTour, graph);

	// Output Results
	std::cout << "Approximate TSP Tour: ";
	for (int node : tspTour) std::cout << node << " ";
	std::cout << std::endl;
	std::cout << "Approximate Cost: " << approxCost << std::endl;

	return 0;
}
