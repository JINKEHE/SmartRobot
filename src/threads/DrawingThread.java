package threads;
import main.SmartRobot;

// this thread is used to draw the occupancy map on the robot's screen
public class DrawingThread extends Thread {
	private SmartRobot myRobot;
	// the time interval for the map to refresh
	private int delay;
	// I would suggest the delay to be lower than 1000
	public DrawingThread(SmartRobot myRobot, int delay) {
		this.myRobot = myRobot;
		this.delay = delay;
		// the running of this thread would not influence the exit of the program
		this.setDaemon(true);
	}

	public void run() {
		while (true) {
			myRobot.drawMap();
			try {
				Thread.sleep(delay);
			} catch (InterruptedException e) {
				myRobot.server.sendToClient("An error occurred in Drawing Thread.\n");
				myRobot.server.sendToClient(e.getMessage()+"\n");
			}
		}
	}
}
