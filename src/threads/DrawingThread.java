package threads;
import main.SmartRobot;

// this thread is used to draw the occupancy map on the robot's screen
public class DrawingThread extends Thread {
	private SmartRobot myRobot;
	// the time interval for the map to refresh
	private int delay;

	public DrawingThread(SmartRobot myRobot, int delay) {
		this.myRobot = myRobot;
		this.delay = delay;
		this.setDaemon(true);
	}

	public void run() {
		while (true) {
			myRobot.drawMap();
			try {
				// wait for some time
				Thread.sleep(delay);
			} catch (InterruptedException e) {
				// pass
			}
		}
	}
}
