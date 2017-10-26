import lejos.hardware.Button;
import lejos.hardware.Keys;

public class ThreadToClose extends Thread {
	private SmartRobot robot;
	public ThreadToClose(SmartRobot robot){
		this.robot = robot;
	}
	public void run(){
		while (true) {
			if (Button.getButtons() == Keys.ID_ESCAPE){
				robot.stop();
			}
		}
	}
}
