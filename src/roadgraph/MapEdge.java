package roadgraph;

/**
 * @author father
 */
class MapEdge {
    private MapNode from;
    private MapNode to;
    private String roadName;
    private String roadType;
    private double length;

    public MapEdge(MapNode from, MapNode to, String roadName, String roadType, double length) {
        this.from = from;
        this.to = to;
        this.roadName = roadName;
        this.roadType = roadType;
        this.length = length;
    }

    public MapNode getFrom() {
        return from;
    }

    public MapNode getTo() {
        return to;
    }

    public String getRoadName() {
        return roadName;
    }

    public String getRoadType() {
        return roadType;
    }

    public double getLength() {
        return length;
    }
}
