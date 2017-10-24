import java.util.function.IntPredicate;

public class OccupanyGridMap {
	private int height, width;
	private int[][] occupied;
	private int[][] detected;
	private int[][] passed;
	// how to add this into local configuration file
	private static final double occupiedThreshold = 0.25;
	
	public OccupanyGridMap(int h_grids, int w_grids) {
		height = h_grids;
		width = w_grids;
		occupied = new int[height][width];
		detected = new int[height][width];
		colors = new int[height][width];
	}
	
	public double getGrid(int h, int w) {
		return 0.5 * (1 + 1.0*occupied[h][w]/detected[h][w]);
	}
	
	public void update(int[] grid, boolean isOccupied) {
		int h = grid[0];
		int w = grid[1];
		detected[h][w] += 1;
		occupied[h][w] += isOccupied ? 1 : -1;
	}
	
	// for color:
	// 
	public void updateColor(int[] grid, int color) {
		
	}
	
	// -1: Unknown // 0: free // 1: occupied
	public int isOccupied(int h, int w) {
		if (detected[h][w] == 0) {
			return -1;
		} else if (getGrid(h, w) <= occupiedThreshold) {
			return 0;
		} else {
			return 1;
		}
	}
	
	public String print() {
		String str = "";
		for (int i=0; i<occupied.length; i++){
			for (int j=0; j<=occupied[i].length-1;j++){
				if (detected[i][j] == 0){
					str += "x";
				} else {
					str += (int)getGrid(i, j);
				}
				str += " ";
			}
			str += "\n";
		}
		return str;
	}
}
