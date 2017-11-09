package threads;
import lejos.hardware.Button;
import lejos.hardware.Keys;
import main.SmartRobot;

// this thread enables the user to turn off the robot at any moment by clicking the Escape  Button
public class StoppingThread extends Thread {
	private SmartRobot robot;
	public StoppingThread(SmartRobot robot){
		this.robot = robot;
		this.setDaemon(true);
	}
	public void run(){
		while (!robot.isTaskFinished()) {
			if (Button.getButtons() == Keys.ID_ESCAPE){
				// call the robot's stopping method
				robot.stop();
			}
		}
	}
}
