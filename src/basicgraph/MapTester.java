package basicgraph;

import roadgraph.MapGraph;
import util.GraphLoader;

import java.util.*;

/**
 * @author father
 */
public class MapTester {

    {
        System.out.println("Some code started at anonymous block");
    }

    static {
        System.out.println("Static code ran... ");
    }

    public static void main(String[] args) {
        System.out.println("Making the new map...");
        MapGraph theMap = new MapGraph();
        //load files from test source
        System.out.println("Done. \nLoading the map...");
        GraphLoader.loadRoadMap("data/testdata/simpletest.map", theMap);
        //parse it and put into graph class

        System.out.println("Done.");
        // create search path method, define data to get back
        System.out.println("Num nodes[V]: " + theMap.getNumVertices());
        System.out.println("Num edges[E]: " + theMap.getNumEdges());



    }


    public List<Integer> getRoute(Integer s, Integer g) {
        Stack<Integer> onTheWay = new Stack<>();
        HashSet<Integer> visited = new HashSet<>();
        HashMap<Integer, Integer> parent = new HashMap<>();
        List<Integer> route = new ArrayList<>();

        //todo: add assert to get along the
        onTheWay.add(s);


        return route;

    }

}
