package behaviors;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.subsumption.Behavior;
import main.SmartRobot;

public class BackBehavior implements Behavior{
	public boolean suppressed;
	private SmartRobot myRobot;
	private MovePilot pilot;
	
	public BackBehavior(SmartRobot robot, MovePilot pilot) {
		myRobot = robot;
		this.pilot = pilot;
	}
	
	public void suppress() {
		suppressed = true;
	}
	
	public void action() {
		suppressed = false;
		pilot.travel(-5, true);
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
