package behaviors;
import lejos.robotics.subsumption.Behavior;
import main.SmartRobot;

// the normal behavior of the robot
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
		// the robot will keep performing this behavior until the task is finished
		while (!suppressed && !finished) {
			finished = myRobot.forward();
		}
	}

	// since this behavior has the lowest priority, it should always wants to take control
	public boolean takeControl() {
		return true;
	}
}
