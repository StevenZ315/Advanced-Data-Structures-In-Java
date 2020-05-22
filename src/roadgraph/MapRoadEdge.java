package roadgraph;

public class MapRoadEdge 
{
	private MapRoadNode fromNode;
	private MapRoadNode toNode;
	private final String roadName;
	private final String roadType;
	private double length;

	
	public MapRoadEdge(MapRoadNode fromNode, MapRoadNode toNode, 
			String roadName, String roadType, double length) {
		this.fromNode = fromNode;
		this.toNode = toNode;
		this.roadName = roadName;
		this.roadType = roadType;
		this.length = length;

	}
	
	public MapRoadNode getFromNode() {
		return this.fromNode;
	}
	
	public MapRoadNode getToNode() {
		return this.toNode;
	}
	
	public String getRoadName() {
		return this.roadName;
	}
	
	public String getRoadType() {
		return this.roadType;
	}
	
	public double getLength() {
		return this.length;
	}
	
	
	
}
