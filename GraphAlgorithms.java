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
import java.util.*;
import java.util.Random;

public class GraphAlgorithms {

    // Returns a graph object of type <String, object>. Takes filename as argument
    // for graph construction
    public static Graph<String, Object> readData(String filename) throws IOException {

        FileReader fr = new FileReader(filename);   // Create filereader
        BufferedReader br = new BufferedReader(fr); // Create bufferedreader

        Graph<String, Object> newGraph = new Graph<>(); // Create graph object
        HashMap<String, Graph.Vertex<String, Object>> cities = new HashMap<>(); // Create hashmap to hold cities

        br.readLine();  // Read first line to skip headers
        String line = br.readLine();    // Read first line of data
        while (line != null) {  // While line is not null

            String[] contents = line.split(",");    // Split on commas

            String state1 = contents[1];    // Grab first state
            String state2 = contents[3];    // Grab second state

            if (!cities.containsKey(state1))    // If hashmap does not contain first state, add to graph
                cities.put(state1, newGraph.addVertex(state1));
            if (!cities.containsKey(state2))    // If hashmap does not contain second state, add to graph
                cities.put(state2, newGraph.addVertex(state2));

            newGraph.addEdge(cities.get(state1), cities.get(state2), Integer.parseInt(contents[5]));    // Add edge between states
            line = br.readLine();
        }
        br.close();
        return newGraph;
    }

    // Returns a hashmap of the shortest path between all nodes in map
    // Takes a graph and vertex as arguments
    public static <V, E> HashMap<Graph.Vertex<V, E>, Double> shortestPaths(Graph<V, E> g, Graph.Vertex<V, E> source) {

        HashMap<Graph.Vertex<V, E>, Double> distances = new HashMap<>();    // Create a hashmap to hold distances
        for (Graph.Vertex<V, E> vertex : g.getVertices()) { // For each vertex in the graph

            distances.put(vertex, Double.POSITIVE_INFINITY);    // Add the vertex to distances with a distance of infinity
        }
        distances.put(source, 0.0);     // Add the starting vertex with a distance of 0

        PriorityQueue<Graph.Vertex<V, E>> queue = new PriorityQueue<>(new Comparator<Graph.Vertex<V, E>>() {    // PriorityQueue with comparator
            @Override
            public int compare(Graph.Vertex<V, E> o1, Graph.Vertex<V, E> o2) {
                return distances.get(o1).compareTo(distances.get(o2));
            }
        });

        for (Graph.Vertex<V, E> vertex : g.getVertices()) { // For each vertex in the graph, add it to queue
            queue.offer(vertex);
        }

        while (!queue.isEmpty()) { // While the queue is not empty

            Graph.Vertex<V, E> cur = queue.poll(); // Poll the next vertex

            for (Graph.Edge<V, E> edgeOut : cur.edgesOut()) { // For each edge out of cur

                Graph.Vertex<V, E> next = edgeOut.other(cur);   // Get the vertex at the other end of the edge

                double curDistToNext = distances.get(next); // Get the distance to the next vertex
                double newDist = distances.get(cur) + ((Graph.WeightedEdge<V, E>) edgeOut).weight;  // Calculate the new distance
                if (newDist < curDistToNext) {  // If the new distance is less than the current distance

                    distances.put(next, newDist);   // Put the new vertex into distances with the new distance
                    queue.remove(next); // Remove the next vertex from queue
                    queue.offer(next);  // Reinsert the vertex into the queue
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

        System.out.println(output.size());  // Print out output size to track over time

        if (curPath.size() == g.getVertices().size()) { // If the path length is equal to the number of vertices

            ArrayList<Graph.Edge<V, E>> curPathEdges = new ArrayList<>();   // Create arraylist of edges

            for (int i = 0; i < curPath.size() - 1; i++){   // For each vertex in curPath

                // Get vertices
                Graph.Vertex<V,E> v0 = curPath.get(i);
                Graph.Vertex<V,E> v1 = curPath.get(i+1);

                Graph.Edge<V,E> edge = g.getEdge(v0, v1);   // Make edge
                if (edge!= null){
                    curPathEdges.add(edge);     // Add edge to output
                }
            }

            Graph.Edge<V,E> lastEdge = g.getEdge(curPath.get(curPath.size()-1), curPath.get(curPath.size() - curPath.size()));  // Grab last edge
            if (lastEdge != null){  // Add last edge to curPath and output

                curPathEdges.add(lastEdge);
                output.add(curPathEdges);
            }
        }

        else{
            // find the next vertex reachable from the end of curPath, add it to curPath, recurse
            Graph.Vertex<V,E> last = curPath.get(curPath.size()-1);

            for (Graph.Edge <V,E> neighbor: last.edgesOut()){   // For each edge out from last

                Graph.Vertex <V,E> next = neighbor.other(last); // Grab last vertex

                if (!curPath.contains(next)){   // If curPath does not contain next

                    curPath.add(next);  // Add next
                    allHamCycles(g, output, curPath);   // Recurse
                    curPath.remove(curPath.size()-1);   // Remove last vertex
                }
            }
        }
    }

    // Returns the minimum distance Hamiltonian Cycle
    // Takes a graph and vertex as arguments
    public static <V,E> List<Graph.Edge<V, E>> minTSP(Graph<V, E> g, Graph.Vertex<V, E> source){

        Collection<List<Graph.Edge<V, E>>> cycles = allHamCycles(g, source);    // Create collection of all Hamiltonian Cycles

        List<Graph.Edge<V, E>> min = null;  // Set min path to null

        int minWeight = 10000000; // Set min path length to an arbitrary large value

        for (List<Graph.Edge<V, E>> path : cycles){ // For each path in the cycles collection

            int totalWeight = 0;    // Set the weight of the path to 0

            for (Graph.Edge<V, E> edge : path){ // For each edge in the path

                totalWeight += ((Graph.WeightedEdge<V, E>) edge).weight;    // Add the weight of the edge to totalWeight
            }

            if (totalWeight < minWeight){   // If the totalWeight is less than the minWeight

                minWeight = totalWeight;    // Update minWeight to totalWeight
                min = path;     // Set min path to current path
            }
        }
        return min;
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
        Random ran = new Random();

        Graph.Vertex<V,E> start = g.getVertex(ran.nextInt(g.getVertices().size()-1));   // Create reference to first vertex
        Graph.Vertex<V,E> prev = null;   // Create reference to last vertex

        stk.push(start);    // Push start to stack
        seen.add(start);    // Add start to seen

        while (!stk.isEmpty()){

            Graph.Vertex<V,E> cur = stk.pop();  // Pop the next vertex from the stack

            if (prev != null){      // If the last vertex is not null

                if (g.getEdge(prev, cur) != null){  // If the edge exists

                    cycle.add(g.getEdge(prev, cur));     // Add an edge between the last vertex and current vertex
                }
            }

            if (!seen.contains(cur)){       // If the current vertex has not been seen, add it to seen
                    seen.add(cur);
            }

            for (Graph.Edge<V,E> edge : tree){  // For each edge in the tree

                if (edge.vertices().contains(cur)){     // If cur is on one of the ends of the edge

                    Graph.Vertex<V,E> neighbor = edge.other(cur);   // Get the other vertex on the edge
                    if (seen.contains(neighbor)){   // If this vertex is already in seen,skip this iteration

                        continue;
                    }
                    stk.add(neighbor);  // Add the neighbor to the stack
                    seen.add(neighbor); // Add the neighbor to seen
                }
            }
            prev = cur; // Set previous to the current node
        }
        int totalWeight = 0;    // Hold total weight (time) of path
        for (Graph.Edge<V, E> edge : cycle){ // For each edge in the path

                totalWeight += ((Graph.WeightedEdge<V, E>) edge).weight;    // Add the weight of the edge to totalWeight
            }
        System.out.println("Total time in seconds: " + totalWeight);    // Print out the total time of the path
        return cycle;
    }

    // Returns whether a given graph is connected or not
    // Takes a graph and a starting node as arguments
    public static <V, E> boolean isConnected(Graph<V, E> g, Graph.Vertex<V, E> source){

        LinkedList<Graph.Vertex<V,E>> queue = new LinkedList<>();   // Create a linked list as a queue for the vertices
        ArrayList<Graph.Vertex<V,E>> seen = new ArrayList<>();   // Create an ArrayList to hold the seen vertices 

        queue.offer(source);    // Offer the first vertex to the queue
        seen.add(source);   // Add the first vertex to seen

        while (!queue.isEmpty()){   // While the queue is not empty

            Graph.Vertex<V,E> cur = queue.poll();   // Poll the next vertex
            for (Graph.Edge<V,E> edgeOut : cur.edgesOut()){ // For each edge out from cur

                Graph.Vertex<V,E> other = edgeOut.other(cur);   // Get the other vertex from the edge
                if (!seen.contains(other) && !queue.contains(other)){   // If this vertex is not in seen or the queue

                    seen.add(other);    // Add new vertex to seen
                    queue.add(other);   // Add new vertex to queue
                }
            }
        }
        if (seen.size() == g.getVertices().size()){ // If seen is the size of the number of vertices in the graph, return true
            return true;
        }
        else{   // Return false if seen is not the size of the number of vertices in the graph
            return false;
        }
    }

    public static void main(String[] args) throws IOException{
        // Graph<String, Object> graph = new Graph<>();
        // for(int i = 0; i < 15; i++) graph.addVertex("" + i);

        // for(int i = 0; i < 15; i++) graph.addEdge(i, (i + 1) % 5, i + 1);
        // for(int i = 0; i < 15; i++) graph.addEdge(i, (i + 2) % 5, i + 1);

        // Graph.Vertex <String, Object>  start = graph.getVertex(0);

        // System.out.println(graph);
        // System.out.println("-".repeat(25));
        // System.out.println("Shortest Distances: " + shortestPaths(graph, graph.getVertex(0)));

        // System.out.println(mst(graph));

        // System.out.println(allHamCycles(graph, start));


        Graph<String, Object> graph2 = readData("StateData.csv");

        // Graph.Vertex <String, Object>  start = graph2.getVertex(0);

        // System.out.println(minTSP(graph2, start));

        // System.out.println(allHamCycles(graph2, start));

        // System.out.println(mst(graph2));

        System.out.println(tspApprox(graph2));

        // System.out.println(shortestPaths(graph2, start));
        // System.out.println(isConnected(graph2, start));
    }
}