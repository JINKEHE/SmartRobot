package behaviors;
import lejos.robotics.subsumption.Behavior;
import main.SmartRobot;

public class MoveBehavior implements Behavior{
	public boolean suppressed;
	private SmartRobot myRobot;
	
	public MoveBehavior(SmartRobot robot) {
		myRobot = robot;
	}
	
	public void suppress() {
		suppressed = true;
	}
	
	public void action() {
		suppressed = false;
		boolean finished = false;
		while (!suppressed && !finished) {
			finished = myRobot.forward();
		}
	}

	public boolean takeControl() {
		return true;
	}
}
