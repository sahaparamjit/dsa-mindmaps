## Graph Algorithms

##### Here is the Java code for the Graph and Node classes, along with the necessary Edge class to support the graph algorithms mentioned:

```java
import java.util.*;

// Class representing a node in the graph
class Node {
    String name;
    List<Node> neighbors;
    List<Edge> edges;
    boolean visited;
    int distance; // For shortest path algorithms
    int inDegree; // For topological sort

    Node(String name) {
        this.name = name;
        this.neighbors = new ArrayList<>();
        this.edges = new ArrayList<>();
        this.visited = false;
        this.distance = Integer.MAX_VALUE; // Set initial distance to infinity
        this.inDegree = 0;
    }

    void addEdge(Node target, int weight) {
        Edge edge = new Edge(this, target, weight);
        this.edges.add(edge);
        this.neighbors.add(target);
        target.inDegree++;
    }

    @Override
    public String toString() {
        return this.name;
    }
}

// Class representing an edge in the graph
class Edge implements Comparable<Edge> {
    Node source;
    Node target;
    int weight;

    Edge(Node source, Node target, int weight) {
        this.source = source;
        this.target = target;
        this.weight = weight;
    }

    @Override
    public int compareTo(Edge other) {
        return Integer.compare(this.weight, other.weight);
    }
}

// Class representing the graph
class Graph {
    List<Node> nodes;
    List<Edge> edges;

    Graph() {
        this.nodes = new ArrayList<>();
        this.edges = new ArrayList<>();
    }

    void addNode(Node node) {
        this.nodes.add(node);
    }

    void addEdge(Node source, Node target, int weight) {
        Edge edge = new Edge(source, target, weight);
        source.edges.add(edge);
        source.neighbors.add(target);
        target.inDegree++;
        this.edges.add(edge);
    }
}

// Example usage
public class GraphExample {
    public static void main(String[] args) {
        Graph graph = new Graph();
        Node a = new Node("A");
        Node b = new Node("B");
        Node c = new Node("C");
        Node d = new Node("D");

        graph.addNode(a);
        graph.addNode(b);
        graph.addNode(c);
        graph.addNode(d);

        graph.addEdge(a, b, 1);
        graph.addEdge(a, c, 4);
        graph.addEdge(b, c, 2);
        graph.addEdge(c, d, 1);
        graph.addEdge(b, d, 5);

        // Now you can call any graph algorithm on this graph
    }
}
```

##### Here's the with all the graph algorithms


| **Algorithm**                | **Description**                                           | **Steps**                                                                                                                                                                                                                                                                                             | **Java Code**                                                                                                                                                                                                                                                                                                                                                                           | 
|------------------------------|-----------------------------------------------------------|-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| **Depth-First Search (DFS)** | Traverses the graph depth-ward, visiting nodes and backtracking. | 1. If the node is null, return.<br>2. Visit the node.<br>3. Mark the node as visited.<br>4. For each neighbor of the node:<br>&ensp;&ensp;- If the neighbor has not been visited, call DFS on the neighbor.                                                                                                                                   | void DFS(Node node) {<br>&ensp;if (node == null) return;<br>&ensp;visit(node);<br>&ensp;node.visited = true;<br>&ensp;for (Node neighbor : node.neighbors) {<br>&ensp;&ensp;if (!neighbor.visited) {<br>&ensp;&ensp;&ensp;DFS(neighbor);<br>&ensp;&ensp;}<br>&ensp;}<br>}                                                                                                   | 
| **Breadth-First Search (BFS)**| Traverses the graph level by level using a queue.         | 1. Initialize a queue with the start node.<br>2. Mark the start node as visited.<br>3. While the queue is not empty:<br>&ensp;&ensp;- Remove a node from the queue.<br>&ensp;&ensp;- Visit the node.<br>&ensp;&ensp;- For each neighbor of the node:<br>&ensp;&ensp;&ensp;&ensp;- If the neighbor has not been visited, add it to the queue and mark it as visited. | void BFS(Node start) {<br>&ensp;Queue<Node> queue = new LinkedList<>();<br>&ensp;queue.add(start);<br>&ensp;start.visited = true;<br>&ensp;while (!queue.isEmpty()) {<br>&ensp;&ensp;Node node = queue.poll();<br>&ensp;&ensp;visit(node);<br>&ensp;&ensp;for (Node neighbor : node.neighbors) {<br>&ensp;&ensp;&ensp;if (!neighbor.visited) {<br>&ensp;&ensp;&ensp;&ensp;queue.add(neighbor);<br>&ensp;&ensp;&ensp;&ensp;neighbor.visited = true;<br>&ensp;&ensp;&ensp;}<br>&ensp;&ensp;}<br>&ensp;}<br>}|
| **Dijkstra's Algorithm**     | Finds the shortest path from a source to all other vertices. | 1. Initialize a priority queue with the source node (distance = 0).<br>2. While the priority queue is not empty:<br>&ensp;&ensp;- Remove the node with the smallest distance.<br>&ensp;&ensp;- For each neighbor of the node:<br>&ensp;&ensp;&ensp;&ensp;- Calculate the new distance.<br>&ensp;&ensp;&ensp;&ensp;- If the new distance is smaller, update the neighbor's distance and add it to the queue. | void dijkstra(Graph graph, Node source) {<br>&ensp;PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingInt(n -> n.distance));<br>&ensp;source.distance = 0;<br>&ensp;pq.add(source);<br>&ensp;while (!pq.isEmpty()) {<br>&ensp;&ensp;Node current = pq.poll();<br>&ensp;&ensp;for (Edge edge : current.edges) {<br>&ensp;&ensp;&ensp;Node neighbor = edge.target;<br>&ensp;&ensp;&ensp;int newDist = current.distance + edge.weight;<br>&ensp;&ensp;&ensp;if (newDist < neighbor.distance) {<br>&ensp;&ensp;&ensp;&ensp;neighbor.distance = newDist;<br>&ensp;&ensp;&ensp;&ensp;pq.add(neighbor);<br>&ensp;&ensp;&ensp;}<br>&ensp;&ensp;}<br>&ensp;}<br>} |
| **Topological Sort**         | Orders vertices in a Directed Acyclic Graph (DAG).       | 1. Initialize an empty stack.<br>2. For each node in the graph:<br>&ensp;&ensp;- If the node has not been visited, call the utility function.<br>3. Utility function:<br>&ensp;&ensp;- Mark the node as visited.<br>&ensp;&ensp;- For each neighbor of the node:<br>&ensp;&ensp;&ensp;&ensp;- If the neighbor has not been visited, call the utility function.<br>&ensp;&ensp;- Push the node to the stack.<br>4. Print the stack contents as the topological order. | void topologicalSort(Graph graph) {<br>&ensp;Stack<Node> stack = new Stack<>();<br>&ensp;for (Node node : graph.nodes) {<br>&ensp;&ensp;if (!node.visited) {<br>&ensp;&ensp;&ensp;topologicalSortUtil(node, stack);<br>&ensp;&ensp;}<br>&ensp;}<br>&ensp;while (!stack.isEmpty()) {<br>&ensp;&ensp;System.out.print(stack.pop() + " ");<br>&ensp;}<br>}<br>void topologicalSortUtil(Node node, Stack<Node> stack) {<br>&ensp;node.visited = true;<br>&ensp;for (Node neighbor : node.neighbors) {<br>&ensp;&ensp;if (!neighbor.visited) {<br>&ensp;&ensp;&ensp;topologicalSortUtil(neighbor, stack);<br>&ensp;&ensp;}<br>&ensp;}<br>&ensp;stack.push(node);<br>} |
| **Kruskal's Algorithm**      | Finds the Minimum Spanning Tree (MST) for a graph.      | 1. Sort all edges in non-decreasing order of their weight.<br>2. Initialize a disjoint set.<br>3. For each edge in the sorted list:<br>&ensp;&ensp;- If including the edge does not form a cycle, include it in the result and perform a union on the sets of the two vertices.<br>4. Return the result as the MST.                                       | void kruskal(Graph graph) {<br>&ensp;Collections.sort(graph.edges);<br>&ensp;DisjointSet ds = new DisjointSet(graph.nodes.size());<br>&ensp;for (Edge edge : graph.edges) {<br>&ensp;&ensp;int u = ds.find(edge.source);<br>&ensp;&ensp;int v = ds.find(edge.target);<br>&ensp;&ensp;if (u != v) {<br>&ensp;&ensp;&ensp;ds.union(u, v);<br>&ensp;&ensp;&ensp;System.out.println(edge.source + " - " + edge.target);<br>&ensp;&ensp;}<br>&ensp;}<br>}|
| **Floyd-Warshall Algorithm** | Finds shortest paths between all pairs of vertices.     | 1. Initialize the distance matrix with edge weights.<br>2. For each vertex k:<br>&ensp;&ensp;- For each vertex i:<br>&ensp;&ensp;&ensp;&ensp;- For each vertex j:<br>&ensp;&ensp;&ensp;&ensp;&ensp;&ensp;- If dist[i][j] > dist[i][k] + dist[k][j], update dist[i][j] to dist[i][k] + dist[k][j].                                                                             | void floydWarshall(int[][] graph) {<br>&ensp;int V = graph.length;<br>&ensp;int[][] dist = new int[V][V];<br>&ensp;for (int i = 0; i < V; i++) {<br>&ensp;&ensp;for (int j = 0; j < V; j++) {<br>&ensp;&ensp;&ensp;dist[i][j] = graph[i][j];<br>&ensp;&ensp;}<br>&ensp;}<br>&ensp;for (int k = 0; k < V; k++) {<br>&ensp;&ensp;for (int i = 0; i < V; i++) {<br>&ensp;&ensp;&ensp;for (int j = 0; j < V; j++) {<br>&ensp;&ensp;&ensp;&ensp;if (dist[i][j] > dist[i][k] + dist[k][j]) {<br>&ensp;&ensp;&ensp;&ensp;&ensp;dist[i][j] = dist[i][k] + dist[k][j];<br>&ensp;&ensp;&ensp;&ensp;}<br>&ensp;&ensp;&ensp;}<br>&ensp;&ensp;}<br>&ensp;}<br>} |
| **Bellman-Ford Algorithm**   | Finds shortest paths from a source to all vertices, detecting negative cycles. | 1. Initialize distances from the source to all vertices as infinity, except the source (distance = 0).<br>2. For each vertex, apply relaxation for all edges V-1 times:<br>&ensp;&ensp;- If the distance to the destination can be shortened by taking the edge, update the destination's distance.<br>3. Check for negative weight cycles by iterating over all edges once more to see if any distance can be further shortened. | void bellmanFord(Graph graph, Node source) {<br>&ensp;int V = graph.nodes.size();<br>&ensp;int[] dist = new int[V];<br>&ensp;Arrays.fill(dist, Integer.MAX_VALUE);<br>&ensp;dist[source.id] = 0;<br>&ensp;for (int i = 0; i < V - 1; i++) {<br>&ensp;&ensp;for (Edge edge : graph.edges) {<br>&ensp;&ensp;&ensp;if (dist[edge.source.id] != Integer.MAX_VALUE && dist[edge.source.id] + edge.weight < dist[edge.target.id]) {<br>&ensp;&ensp;&ensp;&ensp;dist[edge.target.id] = dist[edge.source.id] + edge.weight;<br>&ensp;&ensp;&ensp;}<br>&ensp;&ensp;}<br>&ensp;}<br>&ensp;for (Edge edge : graph.edges) {<br>&ensp;&ensp;if (dist[edge.source.id] != Integer.MAX_VALUE && dist[edge.source.id] + edge.weight < dist[edge.target.id]) {<br>&ensp;&ensp;&ensp;System.out.println("Graph contains negative weight cycle");<br>&ensp;&ensp;&ensp;return;<br>&ensp;&ensp;}<br>&ensp;}<br>} |
| **Kahn's Algorithm**         | Topological sorting of a directed acyclic graph (DAG).  | 1. Compute in-degree (number of incoming edges) for each node.<br>2. Initialize a queue with all nodes having in-degree of 0.<br>3. While the queue is not empty:<br>&ensp;&ensp;- Remove a node from the queue.<br>&ensp;&ensp;- Append it to the topological order.<br>&ensp;&ensp;- Decrease the in-degree of all its neighbors by 1.<br>&ensp;&ensp;- If any neighbor's in-degree becomes 0, add it to the queue.<br>4. If all nodes are processed, return the topological order; otherwise, there is a cycle. | void kahnTopologicalSort(Graph graph) {<br>&ensp;Queue<Node> queue = new LinkedList<>();<br>&ensp;for (Node node : graph.nodes) {<br>&ensp;&ensp;if (node.inDegree == 0) {<br>&ensp;&ensp;&ensp;queue.add(node);<br>&ensp;&ensp;}<br>&ensp;}<br>&ensp;while (!queue.isEmpty()) {<br>&ensp;&ensp;Node node = queue.poll();<br>&ensp;&ensp;System.out.print(node + " ");<br>&ensp;&ensp;for (Node neighbor : node.neighbors) {<br>&ensp;&ensp;&ensp;neighbor.inDegree--;<br>&ensp;&ensp;&ensp;if (neighbor.inDegree == 0) {<br>&ensp;&ensp;&ensp;&ensp;queue.add(neighbor);<br>&ensp;&ensp;&ensp;}<br>&ensp;&ensp;}<br>&ensp;}<br>} |
| **Prim's Algorithm**         | Finds the Minimum Spanning Tree (MST) for a graph.      | 1. Initialize a priority queue with the source node (weight = 0).<br>2. While the priority queue is not empty:<br>&ensp;&ensp;- Remove the node with the smallest weight.<br>&ensp;&ensp;- Add the node to the MST.<br>&ensp;&ensp;- For each neighbor of the node:<br>&ensp;&ensp;&ensp;&ensp;- If the neighbor is not in the MST and the edge weight is smaller than the current known weight, update the neighbor's weight and add it to the priority queue. | void primMST(Graph graph, Node source) {<br>&ensp;PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingInt(n -> n.weight));<br>&ensp;source.weight = 0;<br>&ensp;pq.add(source);<br>&ensp;Set<Node> inMST = new HashSet<>();<br>&ensp;while (!pq.isEmpty()) {<br>&ensp;&ensp;Node current = pq.poll();<br>&ensp;&ensp;if (inMST.contains(current)) continue;<br>&ensp;&ensp;inMST.add(current);<br>&ensp;&ensp;for (Edge edge : current.edges) {<br>&ensp;&ensp;&ensp;Node neighbor = edge.target;<br>&ensp;&ensp;&ensp;if (!inMST.contains(neighbor) && edge.weight < neighbor.weight) {<br>&ensp;&ensp;&ensp;&ensp;neighbor.weight = edge.weight;<br>&ensp;&ensp;&ensp;&ensp;pq.add(neighbor);<br>&ensp;&ensp;&ensp;}<br>&ensp;&ensp;}<br>&ensp;}<br>} |
