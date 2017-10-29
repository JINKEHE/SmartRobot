package threads;
import lejos.hardware.Button;
import lejos.hardware.Keys;
import main.SmartRobot;

// this thread is used to turn off the robot at any moment by clicking the ESCAPE button
public class StoppingThread extends Thread {
	private SmartRobot robot;
	public StoppingThread(SmartRobot robot){
		this.robot = robot;
		this.setDaemon(true);
	}
	public void run(){
		while (true) {
			if (Button.getButtons() == Keys.ID_ESCAPE){
				// call the robot's closing method
				robot.stop();
			}
		}
	}
}
