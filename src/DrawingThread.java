public class DrawingThread extends Thread {
	private SmartRobot myRobot;
	private int delay;

	public DrawingThread(SmartRobot myRobot, int delay) {
		this.myRobot = myRobot;
		this.delay = delay;
		this.setDaemon(false);
	}

	public void run() {
		try {
			while (true) {
				myRobot.drawMap();
				Thread.sleep(delay);
			}
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
	}
}
