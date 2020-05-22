package roadgraph;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import geography.GeographicPoint;

@SuppressWarnings("serial")
public class MapRoadNode extends GeographicPoint implements Comparable<MapRoadNode>
{
	// private int inDegree;
	// private List<MapRoadEdge> fromEdges = new ArrayList<>();
	private List<MapRoadEdge> outEdges = new ArrayList<>();
	private HashMap<MapRoadNode, java.lang.Double> distanceFromNodes = new HashMap<MapRoadNode, java.lang.Double>();
	private double distanceFromStart = java.lang.Double.POSITIVE_INFINITY;
	private double estimateToGoal = 0;
	
	public MapRoadNode(double latitude, double longitude) {
		super(latitude, longitude);
	}
	
	// construct MapRoadNode from GeographicPoint.
	public static MapRoadNode fromGeographicPoint(GeographicPoint p) {
		return new MapRoadNode(p.x, p.y);
	}
	
	public boolean addRoadTo(MapRoadEdge road) {
		if (outEdges.contains(road)) {
			return false;
		}
		outEdges.add(road);
		return true;
	}
	
	public List<MapRoadEdge> getOutEdges() {
		return this.outEdges;
	}
	
	public int getNumOutEdges() {
		return this.outEdges.size();
	}
	
	public void setDistanceFromStart(double distance) {
		this.distanceFromStart = distance;
	} 
	
	public double getDistanceFromStart() {
		return this.distanceFromStart;
	} 
	
	public void setEstimateToGoal(double distance) {
		this.estimateToGoal = distance;
	} 
	
	public double getDistanceToGoal() {
		return this.estimateToGoal;
	}
	
	
	public double getDistanceFromNode(MapRoadNode other) {
		if (distanceFromNodes.containsKey(other)) {
			return distanceFromNodes.get(other);
		} else {
			return java.lang.Double.POSITIVE_INFINITY;
		}
	}
	
	public HashMap<MapRoadNode, java.lang.Double> getDistanceFromNodes() {
		return distanceFromNodes;
	}
	
	
	public void setDistanceFromNode(MapRoadNode other, double distance) {
		distanceFromNodes.put(other, distance);
	}
	
	@Override
	public int compareTo(MapRoadNode other) {
		double result = (this.distanceFromStart - other.distanceFromStart) + 
						(this.estimateToGoal - other.estimateToGoal);
		return result < 0 ? -1 : result == 0 ? 0 : 1;
	}
	

}
