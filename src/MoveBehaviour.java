import lejos.robotics.subsumption.Behavior;

public class MoveBehaviour implements Behavior{
	public boolean suppressed;
	private SimpleRobot myRobot;
	
	public MoveBehaviour(SimpleRobot robot) {
		myRobot = robot;
	}
	
	public void suppress() {
		suppressed = true;
	}
	
	public void action() {
		suppressed = false;
		while (!suppressed) {
			myRobot.forward();
		}
	}

	public boolean takeControl() {
		return true;
	}
}
