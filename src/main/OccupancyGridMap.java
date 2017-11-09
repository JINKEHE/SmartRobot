package main;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.Set;

// the class of Occupancy Grid Map
public class OccupancyGridMap {
	// the height, width of the map
	private int height, width;
	// how many times each grid is seen occupied
	private int[][] occupiedTimes;
	// how many times each grid is seen
	private int[][] detectedTimes;
	// matrix to record whether a grid has been passed through
	// 0 for grids the robot hasn't passed through
	// 1 for grids the robot has passed through
	private int[][] passedBy;
	// the end point (grid with blue color paper)
	private int[] endPoint;
	// whether the end point is found
	private boolean endPointFound = false;
	// the threshold to determine whether a grid is considered as occupied
	private static final double OCCUPIED_THRESHOLD = 0.6;
	
	public OccupancyGridMap(int h_grids, int w_grids) {
		height = h_grids;
		width = w_grids;
		// initialize the matrices with all 0s
		occupiedTimes = new int[height][width];
		detectedTimes = new int[height][width];
		passedBy = new int[height][width];
	}
	
	// get the occupied possibility of a grid
	public double getGrid(int h, int w) {
		return 0.5 * (1 + 1.0*occupiedTimes[h][w]/detectedTimes[h][w]);
	}
	
	/*
	 * is the grid occupied?
	 * 
	 * Return -1 if the grid is unknown
	 * Return  1 if the grid is occupied
	 * Return  0 if the grid is not occupied
	 * 
	 * */
	public int isOccupied(int h, int w) {
		if (detectedTimes[h][w] == 0) {
			return -1;
		} else if (getGrid(h, w) <= OCCUPIED_THRESHOLD) {
			return 0;
		} else {
			return 1;
		}
	}
	
	// update a grid's information
	public void update(int h, int w, boolean isSeenOccupied) {
		if (passedBy[h][w] != 1) {
			detectedTimes[h][w] += 1;
			occupiedTimes[h][w] += isSeenOccupied ? 1 : -1;
		}
	}
	
	// get the end point
	public int[] getEndPoint(){
		return endPoint;
	}
	
	// get the grids that haven't been passed through by the robot
	private ArrayList<int[]> getNotPassedGrids() {
		ArrayList<int[]> notPassedGrids = new ArrayList<int[]>();
		for (int h=0; h<=height-1; h++) {
			for (int w=0; w<=width-1; w++) {
				if (passedBy[h][w] == 0 && isOccupied(h, w)==0)
					notPassedGrids.add(new int[]{h,w});
			}
		}
		return notPassedGrids;
	}
	
	// update the information when a grid is passed by the robot
	public void updatePassed(int h, int w) {
		passedBy[h][w] = 1;
		// if a grid can be reached by the robot, how could it possibly be occupied?
		update(h, w, false); 
		occupiedTimes[h][w] = -1;
		detectedTimes[h][w] = 1;
	}
	
	// update the information of the end point when it is found
	public void updateEndPoint(int[] endGrid) {
		this.endPoint = endGrid;
		endPointFound = true;
	}
	
	// whether the end point has been found in the previous exploration?
	public boolean isEndGridFound() {
		return endPointFound;
	}
	
	// is the map complete? 
	// return true if every grid has been seen at least once
	public boolean isMapFinished() {
		boolean finished = true;
		for (int h=0; h<=height-1; h++) {
			for (int w=0; w<=width-1; w++) {
				if (detectedTimes[h][w] == 0) {
					finished = false;
				}
			}
		}
		return finished;
	}
	

	
	// calculate the manhattan distance between two grids
	// the heuristic estimation used in A* search
	private int estimate(int[] start, int[] goal) {
		int h = Math.abs(start[0] - goal[0]);
		int w = Math.abs(start[1] - goal[1]);
		return h + w;
	}
	
	// my A* path finding algorithm implementation
	public LinkedList<int[]> aStarPathFinding(int[] start, int[] goal) {
		LinkedList<int[]> openList = new LinkedList<int[]>();
		Set<String> closedList = new HashSet<String>();
		HashMap<String, Integer> gValues = new HashMap<String,Integer>();
		HashMap<String, int[]> parents = new HashMap<String, int[]>();
		boolean found = false;
		openList.add(start);
		gValues.put(toString(start), 0);
		parents.put(toString(start), null);
		while (!(found || openList.isEmpty())) {
			int[] minialGrid = openList.get(0);
			for (int[] grid: openList){
				if (gValues.get(toString(grid)) + estimate(grid, goal) < gValues.get(toString(minialGrid)) + estimate(minialGrid, goal)){
					minialGrid = grid;
				}
			}
			openList.remove(minialGrid);
			closedList.add(toString(minialGrid));
			for (int[] g: new int[][]{{-1,0}, {0,-1}, {0,1}, {1,0}}){
				int h = minialGrid[0] + g[0];
				int w = minialGrid[1] + g[1];
				int[] newNode = {h,w};
				if (h<=height-1 && w<=width-1 && h>=0 && w>=0 && !closedList.contains(toString(newNode)) && !gValues.containsKey(toString(newNode)) && isOccupied(h, w)==0) {
					parents.put(toString(newNode), minialGrid);
					if (toString(goal).equals(toString(newNode))) {
						found = true;
						break;
					} else {
						openList.addFirst(newNode);
						gValues.put(toString(newNode), 1+gValues.get(toString(minialGrid)));
					}
				}
			}
			gValues.remove(minialGrid);
		}
		int[] grid = goal;
		LinkedList<int[]> path = new LinkedList<int[]>();
		while (!toString(grid).equals(toString(start))) {
			path.addFirst(grid);
			grid = parents.get(toString(grid));
		}
		return path;
	}
	
	/*
	 * a grid (array) -> "(h,w)" (String)
	 * 
	 * Why do I need this method?
	 * 
	 * Because if I use array as the key in the map, I can't get the same value
	 * using another array with the same elements. This is because an array is 
	 * an object. Therefore, I choose to convert each grid(array) into a String
	 * and use the String as key so different arrays which represent the same
	 * grid can get the same value in the map after they are converted to Strings.
	 * */
	public String toString(int[] a){
		return "("+a[0]+","+a[1]+")";
	}
	
	// Paths can be generated by A* search algorithm but which grid to go to first?
	// The solution here is that we generate a path from the current location to every
	// grid whose color is unknown except obstacles. Then among these paths, we find a
	// path that can pass the most unknown grids and then we still haven't known the end
	// point after the path, we find a next path. The advantage is that this path can
	// gurantee to find an optimum path to a grid whose color is unknown. Moreover,
	// to save energy for moving, it can explore the largest number of unknown grids in a path.
	public LinkedList<int[]> findTheBestPathToExplore(int[] robotLocation){
		ArrayList<int[]> unknownGrids = getNotPassedGrids();
		int bestCount = -1;
		LinkedList<int[]> bestPath = new LinkedList<int[]>();
		for (int[] gridToReach: unknownGrids){
			LinkedList<int[]> path = aStarPathFinding(robotLocation, gridToReach);
			int count = 0;
			for (int[] gridToPass: path){
				for (int[] gridInList: unknownGrids){
					if (toString(gridInList).equals(toString(gridToPass))){
						count += 1;
					}
				}
			}
			if (bestCount == -1 || count > bestCount){
				bestCount = count;
				bestPath = path;
			}
		}
		return bestPath;
	}
	
	// print the map to the console
	// used to debug
	private String print() {
		String str = "";
		System.out.println(occupiedTimes.length);
		for (int i=occupiedTimes.length-1; i>=0;i--){
			for (int j=occupiedTimes[0].length-1; j>=0; j--){
				if (detectedTimes[i][j] == 0){
					str += "x";
				} else {
					str += isOccupied(i, j);
				}
				str += " ";
			}
			str += "\n";
		}
		return str;
	}
	
	// print the path
	// used to debug
	public void printPath(LinkedList<int[]> path){
		for(int i=0; i<=path.size()-1; i++){
			System.out.print(toString(path.get(i))+"->");
		}
		System.out.println();
	}
	
	// testing of the map and the A* search algorithm
	public static void main(String[] args) {
		OccupancyGridMap map = new OccupancyGridMap(7, 6);
		for (int i=0; i<=6; i++){
			for (int j=0; j<=5; j++){
				map.update(i, j, false);
			}
		}
		map.update(0, 4, true);
		map.update(4, 4, true);
		map.update(3, 5, true);
		System.out.println(map.print());
		map.printPath(map.findTheBestPathToExplore(new int[]{4,5}));
	}
}
