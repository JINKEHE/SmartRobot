package behaviors;

import lejos.hardware.Button;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.subsumption.Behavior;
import main.SmartRobot;

// the robot will perform this behavior when it runs into something
// ideally, the arbitrary will never turn to this behavior
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
		Button.waitForAnyPress();
		// try to do something to save itself...
		// we hope the robot would never go to this step
		while(!suppressed && pilot.isMoving()) {
			if (myRobot.isLeftBumpPressed() && !myRobot.isRightBumpPressed()) {
				pilot.travel(-6);
				pilot.rotate(10);
				pilot.travel(16);
			} else if (myRobot.isRightBumpPressed() && !myRobot.isLeftBumpPressed()) {
				pilot.travel(-6);
				pilot.rotate(-10);
				pilot.travel(16);
			} else if (myRobot.isLeftBumpPressed() && myRobot.isRightBumpPressed()) {
				pilot.travel(-10);
			}
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
