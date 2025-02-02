#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <stack>

using namespace std;

const int INF = numeric_limits<int>::max();

// Define a type for an edge: (neighbor, weight)
typedef pair<int, int> Edge;

// Dijkstra's algorithm implementation
// Returns the minimum distances from 'start' to every other node and fills 'previous'
// with the previous node on the shortest path.
vector<int> dijkstra(int start, const vector<vector<Edge>>& graph, vector<int>& previous) {
    int n = graph.size();
    vector<int> dist(n, INF);
    previous.assign(n, -1);
    dist[start] = 0;

    // priority_queue stores pairs of (distance, node)
    priority_queue<Edge, vector<Edge>, greater<Edge>> pq;
    pq.push({0, start});

    while (!pq.empty()) {
        int currentDist = pq.top().first;
        int currentNode = pq.top().second;
        pq.pop();

        // If we have already found a better path before, skip processing.
        if (currentDist > dist[currentNode])
            continue;

        // Check neighbors
        for (const Edge& edge : graph[currentNode]) {
            int neighbor = edge.first;
            int weight = edge.second;
            int newDist = currentDist + weight;
            if (newDist < dist[neighbor]) {
                dist[neighbor] = newDist;
                previous[neighbor] = currentNode;
                pq.push({newDist, neighbor});
            }
        }
    }
    return dist;
}

// Helper function to reconstruct the path from start to end using the 'previous' vector.
vector<int> reconstructPath(int start, int end, const vector<int>& previous) {
    vector<int> path;
    for (int at = end; at != -1; at = previous[at]) {
        path.push_back(at);
    }
    // The path is built backwards, so reverse it
    reverse(path.begin(), path.end());

    // Check if the path starts with the start node; if not, no valid path exists.
    if (!path.empty() && path[0] == start)
        return path;
    else
        return vector<int>();  // return an empty path if there is no connection.
}

int main() {
    // Hard-coded graph.
    // Graph is represented as an adjacency list.
    // For example, graph[node] = vector of (neighbor, weight)
    //
    // Example Graph:
    // 0: (1, 4), (2, 1)
    // 1: (3, 1)
    // 2: (1, 2), (3, 5)
    // 3: (no outgoing edges)
    vector<vector<Edge>> graph = {
        { {1, 4}, {2, 1} },   // Neighbors of node 0
        { {3, 1} },           // Neighbors of node 1
        { {1, 2}, {3, 5} },    // Neighbors of node 2
        { }                   // Neighbors of node 3
    };

    int numNodes = graph.size();

    // Ask user for starting and ending nodes
    int start, end;
    cout << "Enter the starting node (0 to " << numNodes - 1 << "): ";
    cin >> start;
    cout << "Enter the ending node (0 to " << numNodes - 1 << "): ";
    cin >> end;

    // Basic input validation.
    if (start < 0 || start >= numNodes || end < 0 || end >= numNodes) {
        cout << "Invalid node number. Please restart and enter a valid node." << endl;
        return 1;
    }

    vector<int> previous;
    vector<int> distances = dijkstra(start, graph, previous);

    if (distances[end] == INF) {
        cout << "There is no path from node " << start << " to node " << end << "." << endl;
    } else {
        cout << "The cost of the shortest path from node " << start << " to node " << end << " is: " 
             << distances[end] << endl;

        vector<int> path = reconstructPath(start, end, previous);
        cout << "The shortest path is: ";
        for (size_t i = 0; i < path.size(); i++) {
            cout << path[i];
            if (i != path.size() - 1)
                cout << " -> ";
        }
        cout << endl;
    }

    return 0;
}
