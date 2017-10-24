import lejos.robotics.navigation.MovePilot;
import lejos.robotics.subsumption.Behavior;

public class AvoidBehaviour implements Behavior{
	public boolean suppressed;
	private SimpleRobot myRobot;
	
	public AvoidBehaviour(SimpleRobot robot, MovePilot pilot) {
		myRobot = robot;
	}
	
	public void suppress() {
		suppressed = true;
	}
	
	public void action() {
		suppressed = false;
		while (!suppressed) {
			myRobot.back();
		}
	}

	@Override
	public boolean takeControl() {
		return false;
	}
}
