/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which reprsents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
package roadgraph;


import java.util.List;
import java.util.PriorityQueue;
import java.util.Queue;
import java.util.Set;
import java.util.Stack;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.function.Consumer;

import geography.GeographicPoint;
import util.GraphLoader;


/**
 * @author UCSD MOOC development team and YOU
 * 
 * A class which represents a graph of geographic locations
 * Nodes in the graph are intersections between 
 *
 */
public class MapGraph {
	//TODO: Add your member variables here in WEEK 3
	private int numVertices;
	private int numEdges;
	private HashMap<GeographicPoint, MapRoadNode> vertices;
	private int nodeRemoved;
	
	
	// Speed limit.
	private HashMap<String, Double> speedLimitRules = new HashMap<String, Double>();
	private static double AVERAGE_SPEED = 50;
	
	/** 
	 * Create a new empty MapGraph 
	 */
	public MapGraph()
	{
		// TODO: Implement in this constructor in WEEK 3
		numVertices = 0;
		numEdges = 0;
		vertices = new HashMap<GeographicPoint, MapRoadNode>();
		nodeRemoved = 0;
		
		// Initialize the speed limits.
		speedLimitRules.put("motorway", 70.0);
		speedLimitRules.put("motorway_link", 70.0);
		speedLimitRules.put("primary", 70.0);
		speedLimitRules.put("secondary", 55.0);
		speedLimitRules.put("residential", 25.0);
		speedLimitRules.put("unclassified", 15.0);
		speedLimitRules.put("truck", 45.0);
		speedLimitRules.put("other", 25.0);

	}
	
	/**
	 * Get the number of vertices (road intersections) in the graph
	 * @return The number of vertices in the graph.
	 */
	public int getNumVertices()
	{
		//TODO: Implement this method in WEEK 3
		return numVertices;
	}
	
	/**
	 * Return the intersections, which are the vertices in this graph.
	 * @return The vertices in this graph as GeographicPoints
	 */
	public Set<GeographicPoint> getVertices()
	{
		//TODO: Implement this method in WEEK 3
		return vertices.keySet();
	}
	
	/**
	 * Get the number of road segments in the graph
	 * @return The number of edges in the graph.
	 */
	public int getNumEdges()
	{
		//TODO: Implement this method in WEEK 3
		return numEdges;
	}

	
	
	/** Add a node corresponding to an intersection at a Geographic Point
	 * If the location is already in the graph or null, this method does 
	 * not change the graph.
	 * @param location  The location of the intersection
	 * @return true if a node was added, false if it was not (the node
	 * was already in the graph, or the parameter is null).
	 */
	public boolean addVertex(GeographicPoint location)
	{
		// TODO: Implement this method in WEEK 3
		if (location == null) {
			return false;
		}
		
		MapRoadNode locationNode = MapRoadNode.fromGeographicPoint(location);
		if (vertices.containsKey(location)) {
			return false;
		}
		vertices.put(location, locationNode);
		numVertices ++;
		return true;
	}
	
	/**
	 * Adds a directed edge to the graph from pt1 to pt2.  
	 * Precondition: Both GeographicPoints have already been added to the graph
	 * @param from The starting point of the edge
	 * @param to The ending point of the edge
	 * @param roadName The name of the road
	 * @param roadType The type of the road
	 * @param length The length of the road, in km
	 * @throws IllegalArgumentException If the points have not already been
	 *   added as nodes to the graph, if any of the arguments is null,
	 *   or if the length is less than 0.
	 */
	public void addEdge(GeographicPoint from, GeographicPoint to, String roadName,
			String roadType, double length) throws IllegalArgumentException {

		//TODO: Implement this method in WEEK 3
		if (!vertices.containsKey(from) || !vertices.containsKey(to)) {
			throw new IllegalArgumentException("Both points must exist in the graph.");
		}
		if (from == null || to == null || roadName == null || roadType == null || length <= 0) {
			throw new IllegalArgumentException("Invalild edge input.");
		}
		
		MapRoadEdge road = new MapRoadEdge(vertices.get(from), vertices.get(to), roadName, roadType, length);
		vertices.get(from).addRoadTo(road);
		vertices.get(to).setDistanceFromNode(vertices.get(from), road.getLength());
		numEdges ++;

	}
	

	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return bfs(start, goal, temp);
	}
	
	/** Find the path from start to goal using breadth first search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	public List<GeographicPoint> bfs(GeographicPoint start, 
			 					     GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 3
		
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		if (!vertices.containsKey(start) || !vertices.containsKey(goal)) {
			System.out.println("Start or goal node does not exist in the vertices.");
			return new LinkedList<GeographicPoint>();
		}
		
		MapRoadNode startNode = vertices.get(start);
		MapRoadNode goalNode = vertices.get(goal);
		HashMap<MapRoadNode, MapRoadNode> parentMap = new HashMap<MapRoadNode, MapRoadNode>();
		boolean found = bfsSearch(startNode, goalNode, parentMap, nodeSearched);
		
		if(!found) {
			System.out.println("No path exists");
			return new LinkedList<GeographicPoint>();
		}
		
		return constructPath(startNode, goalNode, parentMap);
	}
	
	/** Determine if a path from start to goal using breadth first search.
	 * 	Meanwhile update the parentMap during BFS.
	 * 
	 * @param startNode The starting location
	 * @param goalNode The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @param parentMap Stores connection information between two nodes.
	 * @return Boolean, whether a path exists.
	 */
	private boolean bfsSearch(MapRoadNode startNode, MapRoadNode goalNode, 
			HashMap<MapRoadNode, MapRoadNode> parentMap,
			Consumer<GeographicPoint> nodeSearched) 
	{
		HashSet<MapRoadNode> visited = new HashSet<MapRoadNode>();
		Queue<MapRoadNode> toExplore = new LinkedList<MapRoadNode>();
		toExplore.add(startNode);
		boolean found = false;
		
		while (!toExplore.isEmpty()) {
			MapRoadNode curr = toExplore.remove();
			nodeSearched.accept(curr);
			if (curr == goalNode) {
				found = true;
				break;
			}
			for(MapRoadEdge neighborRoad : curr.getOutEdges()) {
				MapRoadNode neighbor = neighborRoad.getToNode();
				if (!visited.contains(neighbor)) {
					toExplore.add(neighbor);
					visited.add(neighbor);
					parentMap.put(neighbor, curr);
				}
			}
		}
		return found;
	}
	
	/** Construct the path between two nodes.
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param parentMap Stores connection information between two nodes.
	 * @return The list of intersections that form the shortest (unweighted)
	 *   path from start to goal (including both start and goal).
	 */
	private LinkedList<GeographicPoint> constructPath(MapRoadNode start, MapRoadNode goal, 
			HashMap<MapRoadNode, MapRoadNode> parentMap) 
	{
		LinkedList<GeographicPoint> path = new LinkedList<GeographicPoint>();
		MapRoadNode curr = goal;
		while (curr != start) {
			path.addFirst(curr);
			curr = parentMap.get(curr);
		}
		path.addFirst(curr);
		return path;
	}

	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
		// You do not need to change this method.
        Consumer<GeographicPoint> temp = (x) -> {};
        return dijkstra(start, goal, temp);
	}
	
	/** Find the path from start to goal using Dijkstra's algorithm
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> dijkstra(GeographicPoint start, 
										  GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4

		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		nodeRemoved = 0;
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		if (!vertices.containsKey(start) || !vertices.containsKey(goal)) {
			System.out.println("Start or goal node does not exist in the vertices.");
			return new LinkedList<GeographicPoint>();
		}
		
		MapRoadNode startNode = vertices.get(start);
		MapRoadNode goalNode = vertices.get(goal);
		HashMap<MapRoadNode, MapRoadNode> parentMap = new HashMap<MapRoadNode, MapRoadNode>();

		for (MapRoadNode node : vertices.values()) {
			node.setEstimateToGoal(0);
			node.setDistanceFromStart(Double.POSITIVE_INFINITY);
		}
		startNode.setDistanceFromStart(0);
		
		boolean found = dijkstraSearch(startNode, goalNode, parentMap, nodeSearched);
		
		if(!found) {
			System.out.println("No path exists");
			return new LinkedList<GeographicPoint>();
		}
		
		return constructPath(startNode, goalNode, parentMap);
	}

	
	private boolean dijkstraSearch(MapRoadNode startNode, MapRoadNode goalNode, 
			HashMap<MapRoadNode, MapRoadNode> parentMap,
			Consumer<GeographicPoint> nodeSearched) 
	{
		HashSet<MapRoadNode> visited = new HashSet<>();
		PriorityQueue<MapRoadNode> toExplore = new PriorityQueue<>();
		toExplore.add(startNode);
		boolean found = false;
		
		while (!toExplore.isEmpty()) {
			MapRoadNode curr = toExplore.remove();
			nodeRemoved ++;
			if (visited.contains(curr)) {
				continue;
			}
			visited.add(curr);			
			// System.out.println("visiting" + curr);
			nodeSearched.accept(curr);
			if (curr.equals(goalNode)) {
				found = true;
				break;
			}
			for(MapRoadEdge neighborRoad : curr.getOutEdges()) {
				MapRoadNode neighbor = neighborRoad.getToNode();
				
				// consider speed limit based on road type.
				String roadType = neighborRoad.getRoadType();
				double speedLimit;
				if (this.speedLimitRules.containsKey(roadType)) {
					speedLimit = this.speedLimitRules.get(roadType);
				} else {
					speedLimit = this.speedLimitRules.get("other");
				}
				
				if (!visited.contains(neighbor)) {
					// update distance from start if necessary.
					double oldMinutesFromStart = neighbor.getDistanceFromStart();
					double newMinutesFromStart = curr.getDistanceFromStart() + 60 * neighbor.getDistanceFromNode(curr)/speedLimit;
					if (oldMinutesFromStart > newMinutesFromStart) {
						neighbor.setDistanceFromStart(newMinutesFromStart);
						parentMap.put(neighbor, curr);
						toExplore.add(neighbor);
						// System.out.println("Adding to list: " + neighbor);
					}
				}
			}
		}
		System.out.println("Number of nodes visited: " + nodeRemoved);
		return found;
	}
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, GeographicPoint goal) {
		// Dummy variable for calling the search algorithms
        Consumer<GeographicPoint> temp = (x) -> {};
        return aStarSearch(start, goal, temp);
	}
	
	/** Find the path from start to goal using A-Star search
	 * 
	 * @param start The starting location
	 * @param goal The goal location
	 * @param nodeSearched A hook for visualization.  See assignment instructions for how to use it.
	 * @return The list of intersections that form the shortest path from 
	 *   start to goal (including both start and goal).
	 */
	public List<GeographicPoint> aStarSearch(GeographicPoint start, 
											 GeographicPoint goal, Consumer<GeographicPoint> nodeSearched)
	{
		// TODO: Implement this method in WEEK 4
		nodeRemoved = 0;
		// Hook for visualization.  See writeup.
		//nodeSearched.accept(next.getLocation());
		if (start == null || goal == null) {
			System.out.println("Start or goal node is null!  No path exists.");
			return new LinkedList<GeographicPoint>();
		}
		if (!vertices.containsKey(start) || !vertices.containsKey(goal)) {
			System.out.println("Start or goal node does not exist in the vertices.");
			return new LinkedList<GeographicPoint>();
		}
		
		MapRoadNode startNode = vertices.get(start);
		MapRoadNode goalNode = vertices.get(goal);
		HashMap<MapRoadNode, MapRoadNode> parentMap = new HashMap<MapRoadNode, MapRoadNode>();
		
		// key difference from dijkstra. Initialize distance to goal for all nodes.
		for (MapRoadNode node : vertices.values()) {
			node.setEstimateToGoal(60 * node.distance(goal)/AVERAGE_SPEED);  // update to consider speed.
			node.setDistanceFromStart(Double.POSITIVE_INFINITY);
		}
		startNode.setDistanceFromStart(0);
		
		boolean found = dijkstraSearch(startNode, goalNode, parentMap, nodeSearched);
		
		if(!found) {
			System.out.println("No path exists");
			return new LinkedList<GeographicPoint>();
		}
		
		return constructPath(startNode, goalNode, parentMap);
	}

	
	
	public static void main(String[] args)
	{	
		/*
		MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);

		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);
		*/
		
		/*
		System.out.print("Making a new map...");
		MapGraph firstMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", firstMap);
		System.out.println("DONE.");
		
		*/
		// You can use this method for testing.  
		
		
		/* Here are some test cases you should try before you attempt 
		 * the Week 3 End of Week Quiz, EVEN IF you score 100% on the 
		 * programming assignment.
		 */
		
		/*
		MapGraph simpleTestMap = new MapGraph();
		GraphLoader.loadRoadMap("data/testdata/simpletest.map", simpleTestMap);
		
		GeographicPoint testStart = new GeographicPoint(1.0, 1.0);
		GeographicPoint testEnd = new GeographicPoint(8.0, -1.0);
		
		System.out.println("Test 1 using simpletest: Dijkstra should be 9 and AStar should be 5");
		List<GeographicPoint> testroute = simpleTestMap.dijkstra(testStart,testEnd);
		System.out.println();
		List<GeographicPoint> testroute2 = simpleTestMap.aStarSearch(testStart,testEnd);
		
		
		
		MapGraph testMap = new MapGraph();
		GraphLoader.loadRoadMap("data/maps/utc.map", testMap);
		
		// A very simple test using real data
		testStart = new GeographicPoint(32.869423, -117.220917);
		testEnd = new GeographicPoint(32.869255, -117.216927);
		System.out.println("Test 2 using utc: Dijkstra should be 13 and AStar should be 5");
		testroute = testMap.dijkstra(testStart,testEnd);
		System.out.println();
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		/*
		
		// A slightly more complex test using real data
		testStart = new GeographicPoint(32.8674388, -117.2190213);
		testEnd = new GeographicPoint(32.8697828, -117.2244506);
		System.out.println("Test 3 using utc: Dijkstra should be 37 and AStar should be 10");
		testroute = testMap.dijkstra(testStart,testEnd);
		testroute2 = testMap.aStarSearch(testStart,testEnd);
		
		
		*/
		
		/* Use this code in Week 3 End of Week Quiz */
		/*MapGraph theMap = new MapGraph();
		System.out.print("DONE. \nLoading the map...");
		GraphLoader.loadRoadMap("data/maps/utc.map", theMap);
		System.out.println("DONE.");

		GeographicPoint start = new GeographicPoint(32.8648772, -117.2254046);
		GeographicPoint end = new GeographicPoint(32.8660691, -117.217393);
		
		
		List<GeographicPoint> route = theMap.dijkstra(start,end);
		List<GeographicPoint> route2 = theMap.aStarSearch(start,end);

		*/
		
	}
	
}
