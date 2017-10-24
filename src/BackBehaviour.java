import lejos.robotics.navigation.MovePilot;
import lejos.robotics.subsumption.Behavior;

public class BackBehaviour implements Behavior{
	public boolean suppressed;
	private SimpleRobot myRobot;
	private MovePilot pilot;
	
	public BackBehaviour(SimpleRobot robot, MovePilot pilot) {
		myRobot = robot;
		this.pilot = pilot;
	}
	
	public void suppress() {
		suppressed = true;
	}
	
	public void action() {
		suppressed = false;
		pilot.travel(-10, true);
		while(!suppressed && pilot.isMoving()) {
			Thread.yield();
		}
	}

	public boolean takeControl() {
		if (!myRobot.isLeftBumpPressed() && !myRobot.isRightBumpPressed()) {
			return false;
		} else {
			pilot.stop();
			return true;
		}
	}
}
