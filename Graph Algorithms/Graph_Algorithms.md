## 1. Graph Algorithms
### 1.1. Kahn's Algorithm
#### 1.1.1. Description
- Kahn's Algorithm is used for topological sorting of a directed acyclic graph (DAG).
- It works by repeatedly removing nodes with no incoming edges.

#### 1.1.2. Steps
1. Compute in-degree (number of incoming edges) for each node.
2. Initialize a queue with all nodes having in-degree of 0.
3. While the queue is not empty:
   - Remove a node from the queue.
   - Append it to the topological order.
   - Decrease the in-degree of all its neighbors by 1.
   - If any neighbor's in-degree becomes 0, add it to the queue.
4. If all nodes are processed, return the topological order; otherwise, there is a cycle.


```mermaid
sequenceDiagram
    participant Start
    participant Graph
    participant Queue
    participant TopologicalOrder
    participant CycleDetection

    Start ->> Graph: Compute in-degrees of all nodes
    Graph ->> Queue: Initialize with nodes of in-degree 0
    loop Until queue is empty
        Queue ->> Queue: Remove node u from queue
        Queue ->> TopologicalOrder: Append u to topological order
        Queue ->> Graph: For each neighbor v of u
        Graph ->> Graph: Decrease in-degree of v by 1
        Graph ->> Queue: If in-degree of v is 0, add v to queue
    end
    Queue ->> CycleDetection: Check if all nodes are processed
    alt All nodes processed
        CycleDetection ->> TopologicalOrder: Return topological order
    else Cycle detected
        CycleDetection ->> TopologicalOrder: Cycle detected
    end
```


Here's the with all the graph algorithms

| **Algorithm**          | **Description**                                           | **Java Code**                                                                                                                                          |
|------------------------|-----------------------------------------------------------|--------------------------------------------------------------------------------------------------------------------------------------------------------|
| **Depth-First Search (DFS)**    | Traverses the graph depth-ward, visiting nodes and backtracking. | void DFS(Node node) {<br>&ensp;if (node == null) return;<br>&ensp;visit(node);<br>&ensp;node.visited = true;<br>&ensp;for (Node neighbor : node.neighbors) {<br>&ensp;&ensp;if (!neighbor.visited) {<br>&ensp;&ensp;&ensp;DFS(neighbor);<br>&ensp;&ensp;}<br>&ensp;}<br>} |
| **Breadth-First Search (BFS)**   | Traverses the graph level by level using a queue.               | void BFS(Node start) {<br>&ensp;Queue<Node> queue = new LinkedList<>();<br>&ensp;queue.add(start);<br>&ensp;start.visited = true;<br>&ensp;while (!queue.isEmpty()) {<br>&ensp;&ensp;Node node = queue.poll();<br>&ensp;&ensp;visit(node);<br>&ensp;&ensp;for (Node neighbor : node.neighbors) {<br>&ensp;&ensp;&ensp;if (!neighbor.visited) {<br>&ensp;&ensp;&ensp;&ensp;queue.add(neighbor);<br>&ensp;&ensp;&ensp;&ensp;neighbor.visited = true;<br>&ensp;&ensp;&ensp;}<br>&ensp;&ensp;}<br>&ensp;}<br>} |
| **Dijkstra's Algorithm**         | Finds the shortest path from a source to all other vertices.     | void dijkstra(Graph graph, Node source) {<br>&ensp;PriorityQueue<Node> pq = new PriorityQueue<>(Comparator.comparingInt(n -> n.distance));<br>&ensp;source.distance = 0;<br>&ensp;pq.add(source);<br>&ensp;while (!pq.isEmpty()) {<br>&ensp;&ensp;Node current = pq.poll();<br>&ensp;&ensp;for (Edge edge : current.edges) {<br>&ensp;&ensp;&ensp;Node neighbor = edge.target;<br>&ensp;&ensp;&ensp;int newDist = current.distance + edge.weight;<br>&ensp;&ensp;&ensp;if (newDist < neighbor.distance) {<br>&ensp;&ensp;&ensp;&ensp;neighbor.distance = newDist;<br>&ensp;&ensp;&ensp;&ensp;pq.add(neighbor);<br>&ensp;&ensp;&ensp;}<br>&ensp;&ensp;}<br>&ensp;}<br>} |
| **Topological Sort**             | Orders vertices in a Directed Acyclic Graph (DAG).               | void topologicalSort(Graph graph) {<br>&ensp;Stack<Node> stack = new Stack<>();<br>&ensp;for (Node node : graph.nodes) {<br>&ensp;&ensp;if (!node.visited) {<br>&ensp;&ensp;&ensp;topologicalSortUtil(node, stack);<br>&ensp;&ensp;}<br>&ensp;}<br>&ensp;while (!stack.isEmpty()) {<br>&ensp;&ensp;System.out.print(stack.pop() + " ");<br>&ensp;}<br>}<br>void topologicalSortUtil(Node node, Stack<Node> stack) {<br>&ensp;node.visited = true;<br>&ensp;for (Node neighbor : node.neighbors) {<br>&ensp;&ensp;if (!neighbor.visited) {<br>&ensp;&ensp;&ensp;topologicalSortUtil(neighbor, stack);<br>&ensp;&ensp;}<br>&ensp;}<br>&ensp;stack.push(node);<br>} |
| **Kruskal's Algorithm**          | Finds the Minimum Spanning Tree (MST) for a graph.              | void kruskal(Graph graph) {<br>&ensp;Collections.sort(graph.edges);<br>&ensp;DisjointSet ds = new DisjointSet(graph.nodes.size());<br>&ensp;for (Edge edge : graph.edges) {<br>&ensp;&ensp;int u = ds.find(edge.source);<br>&ensp;&ensp;int v = ds.find(edge.target);<br>&ensp;&ensp;if (u != v) {<br>&ensp;&ensp;&ensp;ds.union(u, v);<br>&ensp;&ensp;&ensp;System.out.println(edge.source + " - " + edge.target);<br>&ensp;&ensp;}<br>&ensp;}<br>} |
| **Bellman-Ford Algorithm**       | Finds shortest paths from a source, accommodating negative weights. | void bellmanFord(Graph graph, Node source) {<br>&ensp;source.distance = 0;<br>&ensp;for (int i = 1; i < graph.nodes.size(); ++i) {<br>&ensp;&ensp;for (Edge edge : graph.edges) {<br>&ensp;&ensp;&ensp;if (edge.source.distance + edge.weight < edge.target.distance) {<br>&ensp;&ensp;&ensp;&ensp;edge.target.distance = edge.source.distance + edge.weight;<br>&ensp;&ensp;&ensp;}<br>&ensp;&ensp;}<br>&ensp;}<br>&ensp;for (Edge edge : graph.edges) {<br>&ensp;&ensp;if (edge.source.distance + edge.weight < edge.target.distance) {<br>&ensp;&ensp;&ensp;System.out.println("Graph contains negative weight cycle");<br>&ensp;&ensp;&ensp;return;<br>&ensp;&ensp;}<br>&ensp;}<br>} |
| **Floyd-Warshall Algorithm**     | Finds shortest paths between all pairs of vertices.              | void floydWarshall(int[][] dist) {<br>&ensp;int V = dist.length;<br>&ensp;for (int k = 0; k < V; k++) {<br>&ensp;&ensp;for (int i = 0; i < V; i++) {<br>&ensp;&ensp;&ensp;for (int j = 0; j < V; j++) {<br>&ensp;&ensp;&ensp;&ensp;if (dist[i][k] + dist[k][j] < dist[i][j]) {<br>&ensp;&ensp;&ensp;&ensp;&ensp;dist[i][j] = dist[i][k] + dist[k][j];<br>&ensp;&ensp;&ensp;&ensp;}<br>&ensp;&ensp;&ensp;}<br>&ensp;&ensp;}<br>&ensp;}<br>} |
| **Kahn's Algorithm**             | Topological sorting using in-degree count.                       | void kahnTopologicalSort(Graph graph) {<br>&ensp;Queue<Node> queue = new LinkedList<>();<br>&ensp;for (Node node : graph.nodes) {<br>&ensp;&ensp;if (node.inDegree == 0) {<br>&ensp;&ensp;&ensp;queue.add(node);<br>&ensp;&ensp;}<br>&ensp;}<br>&ensp;while (!queue.isEmpty()) {<br>&ensp;&ensp;Node node = queue.poll();<br>&ensp;&ensp;System.out.print(node + " ");<br>&ensp;&ensp;for (Node neighbor : node.neighbors) {<br>&ensp;&ensp;&ensp;neighbor.inDegree--;<br>&ensp;&ensp;&ensp;if (neighbor.inDegree == 0) {<br>&ensp;&ensp;&ensp;&ensp;queue.add(neighbor);<br>&ensp;&ensp;&ensp;}<br>&ensp;&ensp;}<br>&ensp;}<br>} |
| **Prim's Algorithm**             | Finds the Minimum Spanning Tree (MST) using a greedy approach.   | void primMST(Graph graph) {<br>&ensp;PriorityQueue<Edge> pq = new PriorityQueue<>(Comparator.comparingInt(e -> e.weight));<br>&ensp;Node start = graph.nodes.get(0);<br>&ensp;start.visited = true;<br>&ensp;pq.addAll(start.edges);<br>&ensp;while (!pq.isEmpty()) {<br>&ensp;&ensp;Edge edge = pq.poll();<br>&ensp;&ensp;Node target = edge.target;<br>&ensp;&ensp;if (!target.visited) {<br>&ensp;&ensp;&ensp;target.visited = true;<br>&ensp;&ensp;&ensp;System.out.println(edge.source + " - " + target);<br>&ensp;&ensp;&ensp;pq.addAll(target.edges);<br>&ensp;&ensp;}<br>&ensp;}<br>} |
