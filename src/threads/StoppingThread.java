package threads;
import lejos.hardware.Button;
import lejos.hardware.Keys;
import main.SmartRobot;

public class StoppingThread extends Thread {
	private SmartRobot robot;
	public StoppingThread(SmartRobot robot){
		this.robot = robot;
		this.setDaemon(true);
	}
	public void run(){
		while (true) {
			if (Button.getButtons() == Keys.ID_ESCAPE){
				robot.stop();
			}
		}
	}
}
