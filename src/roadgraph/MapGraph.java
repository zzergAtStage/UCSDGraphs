package roadgraph;

import geography.GeographicPoint;
import util.GraphLoader;

import java.util.*;
import java.util.function.Consumer;

public class MapGraph {

    private static int visitedNodes = 0;
    StackTraceElement[] stackTrace = Thread.currentThread().getStackTrace();
    // Member variables
    private Map<GeographicPoint, MapNode> vertices;
    private int numEdges;

    /**
     * Create a new empty MapGraph
     */
    public MapGraph() {
        vertices = new HashMap<>();
        numEdges = 0;
    }

    public static void main(String[] args) {
        System.out.print("Making a new map...");
        MapGraph firstMap = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
        System.out.println("DONE.");

        // You can use this method for testing.

        MapGraph simpleTestMap = new MapGraph();
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);

        GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
        GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);

        System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
        List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart, testEnd);
        System.out.println("\n Test 1. [dijkstra] Visited nodes are: " + visitedNodes + "\n");
        List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart, testEnd);
        System.out.println("\n Test 1. [aStarSearch] Visited nodes are: " + visitedNodes + "\n");

        MapGraph testMap = new MapGraph();
        GraphLoader.loadRoadMap("data/maps/utc.map", testMap);

        // A very simple test using real data
        testStart = new GeographicPoint(32.869423, -117.220917);
        testEnd = new GeographicPoint(32.869255, -117.216927);
        System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
        testroute = testMap.dijkstra(testStart, testEnd);
        System.out.println("\n Test 2. [dijkstra] Visited nodes are: " + visitedNodes + "\n");
        testroute2 = testMap.aStarSearch(testStart, testEnd);
        System.out.println("\n Test 2. [aStarSearch] Visited nodes are: " + visitedNodes + "\n");

        // A slightly more complex test using real data
        testStart = new GeographicPoint(32.8674388, -117.2190213);
        testEnd = new GeographicPoint(32.8697828, -117.2244506);
        System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
        testroute = testMap.dijkstra(testStart, testEnd);
        System.out.println("\n Test 3. [dijkstra] Visited nodes are: " + visitedNodes + "\n");
        testroute2 = testMap.aStarSearch(testStart, testEnd);
        System.out.println("\n Test 3. [aStarSearch] Visited nodes are: " + visitedNodes + "\n");




        MapGraph theMap = new MapGraph();
        System.out.print("DONE. \nLoading the map...");
        GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
        System.out.println("DONE.");

        GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
        GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

        List<GeographicPoint> route = theMap.dijkstra(start,end);
        System.out.println("\n Grade 1. [dijkstra] Visited nodes are: " + visitedNodes + "\n");
        List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
        System.out.println("\n Grade 1. [aStarSearch] Visited nodes are: " + visitedNodes + "\n");


    }

    /**
     * Get the number of vertices (road intersections) in the graph
     *
     * @return The number of vertices in the graph.
     */
    public int getNumVertices() {
        return vertices.size();
    }

    /**
     * Return the intersections, which are the vertices in this graph.
     *
     * @return The vertices in this graph as GeographicPoints
     */
    public Set<GeographicPoint> getVertices() {
        return vertices.keySet();
    }

    /**
     * Get the number of road segments in the graph
     *
     * @return The number of edges in the graph.
     */
    public int getNumEdges() {
        return numEdges;
    }

    /**
     * Add a node corresponding to an intersection at a Geographic Point
     * If the location is already in the graph or null, this method does
     * not change the graph.
     *
     * @param location The location of the intersection
     * @return true if a node was added, false if it was not (the node
     * was already in the graph, or the parameter is null).
     */
    public boolean addVertex(GeographicPoint location) {
        if (location == null || vertices.containsKey(location)) {
            return false;
        }
        vertices.put(location, new MapNode(location));
        return true;
    }

    /**
     * Adds a directed edge to the graph from pt1 to pt2.
     * Precondition: Both GeographicPoints have already been added to the graph
     *
     * @param from     The starting point of the edge
     * @param to       The ending point of the edge
     * @param roadName The name of the road
     * @param roadType The type of the road
     * @param length   The length of the road, in km
     * @throws IllegalArgumentException If the points have not already been
     *                                  added as nodes to the graph, if any of the arguments is null,
     *                                  or if the length is less than 0.
     */
    public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
                        String roadType, double length) throws IllegalArgumentException {
        if (from == null || to == null || roadName == null || roadType == null || length < 0) {
            throw new IllegalArgumentException();
        }
        if (!vertices.containsKey(from) || !vertices.containsKey(to)) {
            throw new IllegalArgumentException("Both nodes must be in the graph");
        }
        MapNode fromNode = vertices.get(from);
        MapNode toNode = vertices.get(to);
        fromNode.addEdge(new MapEdge(fromNode, toNode, roadName, roadType, length));
        numEdges++;
    }

    /**
     * Find the path from start to goal using breadth first search
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return bfs(start, goal, temp);
    }

    /**
     * Find the path from start to goal using breadth first search
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest (unweighted)
     * path from start to goal (including both start and goal).
     */
    public List<GeographicPoint> bfs(GeographicPoint start,
                                     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        if (start == null || goal == null || !vertices.containsKey(start) || !vertices.containsKey(goal)) {
            return null;
        }

        Queue<GeographicPoint> queue = new LinkedList<>();
        Set<GeographicPoint> visited = new HashSet<>();
        Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();

        queue.add(start);
        visited.add(start);

        boolean found = false;

        while (!queue.isEmpty()) {
            GeographicPoint curr = queue.remove();
            nodeSearched.accept(curr);

            if (curr.equals(goal)) {
                found = true;
                break;
            }

            for (MapEdge edge : vertices.get(curr).getEdges()) {
                GeographicPoint neighbor = edge.getTo().getLocation();
                if (!visited.contains(neighbor)) {
                    queue.add(neighbor);
                    visited.add(neighbor);
                    parentMap.put(neighbor, curr);
                }
            }
        }

        if (!found) {
            return null;
        }

        return reconstructPath(start, goal, parentMap);
    }

    /**
     * Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return dijkstra(start, goal, temp);
    }

    /**
     * Find the path from start to goal using Dijkstra's algorithm
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> dijkstra(GeographicPoint start,
                                          GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        return searchAlgorithm(start, goal, nodeSearched, (p1, p2) -> Double.compare(p1.getDistance(), p2.getDistance()));
    }

    /**
     * Find the path from start to goal using A-Star search
     *
     * @param start The starting location
     * @param goal  The goal location
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
        Consumer<GeographicPoint> temp = (x) -> {
        };
        return aStarSearch(start, goal, temp);
    }

    // Helper methods

    /**
     * Find the path from start to goal using A-Star search
     *
     * @param start        The starting location
     * @param goal         The goal location
     * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
     * @return The list of intersections that form the shortest path from
     * start to goal (including both start and goal).
     */
    public List<GeographicPoint> aStarSearch(GeographicPoint start,
                                             GeographicPoint goal, Consumer<GeographicPoint> nodeSearched) {
        return searchAlgorithm(start, goal, nodeSearched, (p1, p2) -> {
            double p1Dist = p1.getDistance() + p1.getLocation().distance(goal);
            double p2Dist = p2.getDistance() + p2.getLocation().distance(goal);
            return Double.compare(p1Dist, p2Dist);
        });
    }

    private List<GeographicPoint> reconstructPath(GeographicPoint start, GeographicPoint goal, Map<GeographicPoint, GeographicPoint> parentMap) {
        LinkedList<GeographicPoint> path = new LinkedList<>();
        GeographicPoint curr = goal;
        while (curr != null && !curr.equals(start)) {
            path.addFirst(curr);
            curr = parentMap.get(curr);
        }
        if (curr == null) {
            return null;
        }
        path.addFirst(start);
        return path;
    }

    private List<GeographicPoint> searchAlgorithm(GeographicPoint start, GeographicPoint goal, Consumer<GeographicPoint> nodeSearched, Comparator<MapNode> comparator) {
        visitedNodes = 0;
        if (start == null || goal == null || !vertices.containsKey(start) || !vertices.containsKey(goal)) {
            return null;
        }

        PriorityQueue<MapNode> toExplore = new PriorityQueue<>(comparator);
        Set<GeographicPoint> visited = new HashSet<>();
        Map<GeographicPoint, GeographicPoint> parentMap = new HashMap<>();

        for (MapNode node : vertices.values()) {
            node.setDistance(Double.POSITIVE_INFINITY);
        }
        MapNode startNode = vertices.get(start);
        startNode.setDistance(0);
        toExplore.add(startNode);

        boolean found = false;

        while (!toExplore.isEmpty()) {
            MapNode currNode = toExplore.poll();
            if (currNode != null) {
                visitedNodes++;
                stackTrace = Thread.currentThread().getStackTrace();
                if (stackTrace.length > 2) System.out.println(stackTrace[2].getMethodName()
                        + " " + currNode);
            }

            GeographicPoint curr = currNode.getLocation();
            nodeSearched.accept(curr);

            if (!visited.contains(curr)) {
                visited.add(curr);

                if (curr.equals(goal)) {
                    found = true;
                    break;
                }

                for (MapEdge edge : currNode.getEdges()) {
                    MapNode neighborNode = edge.getTo();
                    GeographicPoint neighbor = neighborNode.getLocation();

                    if (!visited.contains(neighbor)) {
                        double newDist = currNode.getDistance() + edge.getLength();
                        if (newDist < neighborNode.getDistance()) {
                            neighborNode.setDistance(newDist);
                            parentMap.put(neighbor, curr);
                            toExplore.add(neighborNode);
                        }
                    }
                }
            }
        }

        if (!found) {
            return null;
        }

        return reconstructPath(start, goal, parentMap);
    }
}



