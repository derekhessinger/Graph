/*
* File: GraphAlgorithms.java
* Derek Hessinger
* CS231 B
* 12/4/22
*/

import java.util.ArrayList;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.Collection;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.PriorityQueue;
import java.util.Stack;

public class GraphAlgorithms {

    // Returns a graph object of type <String, object>. Takes filename as argument
    // for graph construction
    public static Graph<String, Object> readData(String filename) throws IOException {
        FileReader fr = new FileReader(filename);
        BufferedReader br = new BufferedReader(fr);

        Graph<String, Object> newGraph = new Graph<>();
        HashMap<String, Graph.Vertex<String, Object>> cities = new HashMap<>();

        br.readLine();
        String line = br.readLine();
        while (line != null) {
            String[] contents = line.split(",");

            String state1 = contents[1];
            String state2 = contents[2];

            if (!cities.containsKey(state1))
                cities.put(state1, newGraph.addVertex(state1));
            if (!cities.containsKey(state2))
                cities.put(state2, newGraph.addVertex(state2));

            newGraph.addEdge(cities.get(state1), cities.get(state2), Integer.parseInt(contents[5]));
            line = br.readLine();
        }
        br.close();
        return newGraph;
    }

    // Returns a hashmap of the shortest path between all nodes in map
    // Takes a graph and vertex as arguments
    public static <V, E> HashMap<Graph.Vertex<V, E>, Double> shortestPaths(Graph<V, E> g, Graph.Vertex<V, E> source) {
        HashMap<Graph.Vertex<V, E>, Double> distances = new HashMap<>();
        for (Graph.Vertex<V, E> vertex : g.getVertices()) {
            distances.put(vertex, Double.POSITIVE_INFINITY);
        }
        distances.put(source, 0.0);

        PriorityQueue<Graph.Vertex<V, E>> queue = new PriorityQueue<>(new Comparator<Graph.Vertex<V, E>>() {
            @Override
            public int compare(Graph.Vertex<V, E> o1, Graph.Vertex<V, E> o2) {
                return distances.get(o1).compareTo(distances.get(o2));
            }
        });

        for (Graph.Vertex<V, E> vertex : g.getVertices()) {
            queue.offer(vertex);
        }

        while (!queue.isEmpty()) { // O(V) where V is number of vertices
            Graph.Vertex<V, E> cur = queue.poll(); // O(log V)

            for (Graph.Edge<V, E> edgeOut : cur.edgesOut()) { // O(V)
                Graph.Vertex<V, E> next = edgeOut.other(cur);

                double curDistToNext = distances.get(next);
                double newDist = distances.get(cur) + ((Graph.WeightedEdge<V, E>) edgeOut).weight;
                if (newDist < curDistToNext) {
                    distances.put(next, newDist);
                    queue.remove(next); // O(log V) for Java, but could be O(1) if implemented better
                    queue.offer(next);
                }
            }
        }
        return distances;
    }

    // Returns a collection of lists of all possible Hamiltonian Cycles
    // Takes in a graph and starting point vertex ans arguments
    public static <V, E> Collection<List<Graph.Edge<V, E>>> allHamCycles(Graph<V, E> g, Graph.Vertex<V, E> start) {
        Collection<List<Graph.Edge<V, E>>> output = new ArrayList<>();  // Create collection
        List<Graph.Vertex<V, E>> curPath = new ArrayList<Graph.Vertex<V, E>>(); // Create list
        curPath.add(start); // Add the starting vertex to the path
        allHamCycles(g, output, curPath);
        return output;
    }

    // Helper function
    private static <V, E> void allHamCycles(Graph<V, E> g, Collection<List<Graph.Edge<V, E>>> output, List<Graph.Vertex<V, E>> curPath) {
        if (curPath.size() == g.getVertices().size()) { // if the path length is equal to the number of vertices

            ArrayList<Graph.Edge<V, E>> curPathEdges = new ArrayList<>();   // Create arraylist of edges

            for (int i = 0; i < curPath.size() - 1; i++){   // for each vertex in curPath

                // Get vertices
                Graph.Vertex<V,E> v0 = curPath.get(i);
                Graph.Vertex<V,E> v1 = curPath.get(i+1);

                // make edge
                Graph.Edge<V,E> edge = g.getEdge(v0, v1);

                // add edge to output
                curPathEdges.add(edge);
            }

            Graph.Edge<V,E> lastEdge = g.getEdge(curPath.get(curPath.size()-1), curPath.get(curPath.size() - curPath.size()));  // Grab last edge
            if (lastEdge != null){  // Add last edge to curPath and output
                output.add(curPathEdges);
            }
            // curPathEdges.add(lastEdge);
            // output.add(curPathEdges);
        }

        else{
            // find the next vertex reachable from the end of curPath, add it to curPath, recurse
            Graph.Vertex<V,E> last = curPath.get(curPath.size()-1);
            //Collection<Graph.Vertex<V, E>> neighbors = last.neighborsOut();
            for (Graph.Edge <V,E> neighbor: last.edgesOut()){
                Graph.Vertex <V,E> next = neighbor.other(last); // Grab last vertex

                if (!curPath.contains(next)){   // If curPath does not contain next

                    curPath.add(next);  // Add next
                    allHamCycles(g, output, curPath);   // Recurse
                    curPath.remove(curPath.size()-1);   // Remove last vertex
                    System.out.println(curPath.size()); // Print out curPath size
                }
            }
        }
    }

    // Minimum spanning tree, implemented with Prim's algorithm
    // Returns a collection of edges that holds the mst
    public static <V, E> Collection<Graph.Edge<V, E>> mst(Graph<V, E> g){  

        Graph.Vertex<V,E> cur = g.getVertex(0);   // Get first vertex to start
        ArrayList<Graph.Vertex<V,E>> seen = new ArrayList();    // Create ArrayList to hold seen vertices
        Collection<Graph.Edge<V,E>> tree = new ArrayList();  // Create Collection to hold the mst
        seen.add(cur);    // add first vertex to seen

        PriorityQueue<Graph.Edge<V,E>> queue = new PriorityQueue<>(new Comparator<Graph.Edge<V, E>>() { // PriorityQueue to hold edges going out from current vertex

            @Override
            public int compare(Graph.Edge<V, E> o1, Graph.Edge<V, E> o2) {

                if(((Graph.WeightedEdge<V,E>)o1).weight > ((Graph.WeightedEdge<V,E>)o2).weight) return 1;

                else if (((Graph.WeightedEdge<V,E>)o1).weight < ((Graph.WeightedEdge<V,E>)o2).weight) return -1;

                else return 0;
            }
        });

        for (Graph.Edge<V,E> edgeOut : cur.edgesOut()){ // Add all edges out from cur to queue
            queue.add(edgeOut);
        }

        while (seen.size() < g.getVertices().size()){   // While all vertices have not yet been visited

            Graph.Edge<V,E> minEdge = queue.poll(); // Poll the minimum edge from the queue
            for (Graph.Vertex<V,E> v : minEdge.vertices()){ // For each vertex attached to the min edge

                if (seen.contains(v)) continue; // If seen contains v, skip this iteration of the loop 
                tree.add(minEdge); // Add min edge to collection of edges
                seen.add(v); // Add v to seen
                cur = v; // set cur to v to advance cur
                for (Graph.Edge<V,E> edgeOut : cur.edgesOut()){

                    queue.add(edgeOut); // Add all edges out from vertex to queue
                } 
            }
        }
        return tree;    // Return mst of edges
    }

    public static <V, E> Collection<Graph.Edge<V, E>> tspApprox(Graph<V, E> g){

        Collection<Graph.Edge<V,E>> tree = mst(g); // Get an MST of the graph
        ArrayList<Graph.Vertex<V,E>> seen = new ArrayList();    // Create ArrayList to hold seen vertices
        Stack<Graph.Vertex<V,E>> stk = new Stack<>();     // Stack to hold edges in hamiltonian cycle
        List<Graph.Edge<V,E>> cycle = new ArrayList<Graph.Edge<V,E>>(); // Create Arraylist to hold solution cycle

        Graph.Vertex<V,E> start = g.getVertex(0);   // Create reference to first vertex
        Graph.Vertex<V,E> prev = null;   // Create reference to last vertex

        stk.push(start);    // Push start to stack
        seen.add(start);    // Add start to seen

        while (!stk.isEmpty()){

            Graph.Vertex<V,E> cur = stk.pop();

            if (prev != null){      // If the last vertex is not null

                if (g.getEdge(prev, cur) != null){
                    cycle.add(g.getEdge(prev, cur));     // Add an edge between the last vertex and current vertex
                }
            }

            if (!seen.contains(cur)){       // If the current vertex has not been seen, add it to seen
                    seen.add(cur);
            }
            for (Graph.Edge<V,E> edge : tree){

                if (edge.vertices().contains(cur)){

                    Graph.Vertex<V,E> neighbor = edge.other(cur);
                    if (seen.contains(neighbor)){

                        continue;
                    }
                    stk.add(neighbor);
                    seen.add(neighbor);
                }
            }
            prev = cur;
        }
        System.out.println(cycle.size());
        return cycle;
    }


    public static void main(String[] args) throws IOException{
        Graph<String, Object> graph = new Graph<>();
        for(int i = 0; i < 9; i++) graph.addVertex("" + i);

        for(int i = 0; i < 9; i++) graph.addEdge(i, (i + 1) % 5, i + 1);
        for(int i = 0; i < 9; i++) graph.addEdge(i, (i + 2) % 5, i + 1);

        System.out.println(graph);
        System.out.println("-".repeat(25));
        System.out.println("Shortest Distances: " + shortestPaths(graph, graph.getVertex(0)));

        System.out.println(mst(graph));


        Graph<String, Object> graph2 = readData("StateData.csv");

        //Graph.Vertex <String, Object>  start = graph2.getVertex(0);

        //System.out.println(allHamCycles(graph, start));

        //System.out.println(mst(graph2));

        System.out.println(tspApprox(graph2));
    }
}