package roadgraph;

import geography.GeographicPoint;

import java.util.ArrayList;
import java.util.List;
import java.util.stream.Collectors;

/**
 * @author father
 */
public class MapNode {
    private GeographicPoint location;
    private List<MapEdge> edges;
    private double distance;

    public MapNode(GeographicPoint loc) {
        location = loc;
        edges = new ArrayList<>();
        distance = Double.POSITIVE_INFINITY;
    }

    public GeographicPoint getLocation() {
        return location;
    }

    public List<MapEdge> getEdges() {
        return edges;
    }

    public void addEdge(MapEdge edge) {
        edges.add(edge);
    }

    public double getDistance() {
        return distance;
    }

    public void setDistance(double distance) {
        this.distance = distance;
    }

    @Override
    public String toString() {
        return "MapNode{ at location (" +
                location.x + " " + location.y + ") " +
                edges.stream().map(MapEdge::getRoadName).collect(Collectors.joining())
                +"}";
    }
}
