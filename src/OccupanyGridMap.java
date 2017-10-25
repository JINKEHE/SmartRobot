import java.io.FileNotFoundException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.LinkedList;
import java.util.List;
import java.util.Set;
import java.util.function.IntPredicate;

import javax.xml.stream.events.StartDocument;

import lejos.robotics.navigation.SteeringPilot;

public class OccupanyGridMap {
	private int height, width;
	private int[][] occupiedTimes;
	private int[][] detectedTimes;
	private int[][] passedBy;
	private int[] endGrid;
	private double H_MOVE;
	private double W_MOVE;
	private boolean endGridFound = false;
	private static final double occupiedThreshold = 0.25;
	
	public OccupanyGridMap(int h_grids, int w_grids, double H_MOVE, double W_MOVE) {
		height = h_grids;
		width = w_grids;
		this.H_MOVE = H_MOVE;
		this.W_MOVE = W_MOVE;
		occupiedTimes = new int[height][width];
		detectedTimes = new int[height][width];
		passedBy = new int[height][width];
	}
	
	public double getGrid(int h, int w) {
		return 0.5 * (1 + 1.0*occupiedTimes[h][w]/detectedTimes[h][w]);
	}
	
	public void update(int h, int w, boolean isOccupied) {
		detectedTimes[h][w] += 1;
		occupiedTimes[h][w] += isOccupied ? 1 : -1;
	}
	
	public int[] getEndPoint(){
		return endGrid;
	}
	
	public ArrayList<int[]> getNotPassedGrids() {
		ArrayList<int[]> notPassedGrids = new ArrayList<int[]>();
		for (int h=0; h<=height-1; h++) {
			for (int w=0; w<=width-1; w++) {
				if (passedBy[h][w] == 0 && occupiedTimes[h][w] <= 0) {
					notPassedGrids.add(new int[]{h,w});
				}
			}
		}
		return notPassedGrids;
	}
	
	public void updatePassed(int h, int w) {
		update(h, w, false);
		passedBy[h][w] = 1;
		//System.out.println("Robot jsut passed grid: " + h + "," + w);
	}
	
	public void updateEndPoint(int[] endGrid) {
		this.endGrid = endGrid;
		endGridFound = true;
	}
	
	public boolean isEndGridFound() {
		return endGridFound;
	}
	
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
		} else if (getGrid(h, w) <= occupiedThreshold) {
			return 0;
		} else {
			return 1;
		}
	}
	
	public String print() {
		String str = "";
		for (int j=0; j<=occupiedTimes[0].length-1;j++){
			for (int i=0; i<occupiedTimes.length; i++){
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
	
	// estimate by manhattan distance
	public int estimate(int[] start, int[] goal) {
		int h = Math.abs(start[0] - goal[0]);
		int w = Math.abs(start[1] - goal[1]);
		return h + w;
	}
	
	// for now, it will just print the path
	public List aStarPathFinding(int[] start, int[] goal) {
		LinkedList<int[]> openList = new LinkedList<int[]>();
		Set<String> closedList = new HashSet<String>();
		HashMap<String, Integer> gValues = new HashMap<String,Integer>();
		HashMap<String, int[]> parents = new HashMap<String, int[]>();
		boolean found = false;
		openList.add(start);
		gValues.put(toStr(start), 0);
		parents.put(toStr(start), null);
		int i = 0;
		while (!(found || openList.isEmpty())) {
			//System.out.println("Step: " + i++);
			int[] minialGrid = openList.get(0);
			for (int[] grid: openList){
				//System.out.println(toStr(grid)+": "+(gValues.get(toStr(grid))+estimate(grid, goal)));
				if (gValues.get(toStr(grid)) + estimate(grid, goal) < gValues.get(toStr(minialGrid)) + estimate(minialGrid, goal)){
					minialGrid = grid;
				}
			}
			
			openList.remove(minialGrid);
			closedList.add(toStr(minialGrid));
			for (int[] g: new int[][]{{1,0}, {-1,0}, {0,1}, {0,-1}}){
				int h = minialGrid[0] + g[0];
				int w = minialGrid[1] + g[1];
				int[] new_node = {h,w};
				if (h<=height-1 && w<=width-1 && h>=0 && w>=0 && !closedList.contains(toStr(new_node)) && !gValues.containsKey(toStr(new_node)) && isOccupied(h, w)==0) {
					parents.put(toStr(new_node), minialGrid);
					if (gridIsEqual(goal, new_node)){
						found = true;
						break;
					} else {
						openList.addFirst(new_node);
						gValues.put(toStr(new_node), 1+gValues.get(toStr(minialGrid)));
					}
				}
			}
			gValues.remove(minialGrid);
		}
		int[] grid = goal;
		LinkedList<int[]> path = new LinkedList<int[]>();
		System.out.print("Path: ");
		while (grid != null) {
			path.addFirst(grid);
			grid = parents.get(toStr(grid));
		}
		return path;
	}
	
	public String toStr(int[] a){
		return "("+a[0]+","+a[1]+")";
	}
	
	public static boolean gridIsEqual(int[] grid1, int[] grid2){
		if (grid1[0] == grid2[0] && grid1[1] == grid2[1]){
			return true;
		} else {
			return false;
		}
	}
	
	public static int getHeading(double origin){
		int sign = origin > 0 ? 1: -1;
		origin = Math.abs(origin);
		double remainder = origin % 90;
		int num = (int) origin / 90;
		if (remainder > 80) {
			remainder = 90;
		} else {
			remainder = 0;
		}
		return (int)(num*90 + remainder)*sign;
	}
	
	public static void main(String[] args) {
		OccupanyGridMap map = new OccupanyGridMap(5, 4, 0, 0);
		for (int i=0; i<=4; i++){
			for (int j=0; j<=3; j++){
				map.update(i, j, false);
			}
		}
		map.update(0, 2, true);
		map.update(1, 2, true);
		map.update(2, 2, true);
		System.out.println(map.print());
		int[] start = {0,0};
		int[] goal = {0,3};
		map.aStarPathFinding(start, goal);
	}
}
