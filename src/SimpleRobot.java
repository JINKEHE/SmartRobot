import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
import lejos.hardware.sensor.NXTUltrasonicSensor;
import lejos.robotics.SampleProvider;
import lejos.robotics.chassis.Chassis;
import lejos.robotics.chassis.Wheel;
import lejos.robotics.chassis.WheeledChassis;
import lejos.robotics.navigation.MovePilot;
import lejos.hardware.motor.EV3LargeRegulatedMotor;
import lejos.hardware.motor.Motor;
import lejos.hardware.Keys;

public class SimpleRobot {
	private Brick myEV3;
	private EV3TouchSensor leftBump, rightBump;
	private EV3IRSensor irSensor;
	private NXTUltrasonicSensor uSensor;
	private EV3ColorSensor cSensor;
	private SampleProvider leftBumpSP, rightBumpSP, colourSP, irDistSP, ultrasonicDistSP;
	private float[] leftBumpSample, rightBumpSample, distSample, colourSample, irDistSample, ultrasonicDistSample;
	private MovePilot pilot;
	private EV3LargeRegulatedMotor uSensorMotor;

	private void setupIRSensor() {
		irSensor = new EV3IRSensor(myEV3.getPort("S3"));
		irDistSP = irSensor.getDistanceMode();
		irDistSample = new float[irDistSP.sampleSize()];
	}
	
	private void setupUltrasonicSensor() {
		uSensor = new NXTUltrasonicSensor(myEV3.getPort("S3"));
		ultrasonicDistSP = uSensor.getDistanceMode();
		ultrasonicDistSample = new float[ultrasonicDistSP.sampleSize()];
	}
	
	private void setupColorSensor() {
		cSensor = new EV3ColorSensor(myEV3.getPort("S4"));
		colourSP = cSensor.getRGBMode();
		colourSample = new float[colourSP.sampleSize()];
	}
	
	private void setupTouchSensor() {
		leftBump = new EV3TouchSensor(myEV3.getPort("S2"));
		rightBump = new EV3TouchSensor(myEV3.getPort("S1"));
		leftBumpSP = leftBump.getTouchMode();
		rightBumpSP = rightBump.getTouchMode();
		leftBumpSample = new float[leftBumpSP.sampleSize()]; 
		rightBumpSample = new float[rightBumpSP.sampleSize()];
	}
	
	private void setupPilot() {
		Wheel leftWheel = WheeledChassis.modelWheel(Motor.B, 3.3).offset(-10.0);
		Wheel rightWheel = WheeledChassis.modelWheel(Motor.C, 3.3).offset(10.0);
		Chassis myChassis = new WheeledChassis(new Wheel[] { leftWheel, rightWheel }, WheeledChassis.TYPE_DIFFERENTIAL);
		pilot = new MovePilot(myChassis);
	}
	
	public SimpleRobot() {
		myEV3 = BrickFinder.getDefault();
		setupPilot();
		setupTouchSensor();
		setupColorSensor();
		//setupIRSensor();
		setupUltrasonicSensor();
	}

	public void closeRobot() {
		leftBump.close();
		rightBump.close();
		irSensor.close();
		cSensor.close();
	}

	public boolean isLeftBumpPressed() {
		leftBumpSP.fetchSample(leftBumpSample, 0);
		return (leftBumpSample[0] == 1.0);
	}

	public boolean isRightBumpPressed() {
		rightBumpSP.fetchSample(rightBumpSample, 0);
		return (rightBumpSample[0] == 1.0);
	}

	public float getDistance() {
		irDistSP.fetchSample(distSample, 0);
		return distSample[0];
	}

	public float[] getColour() {
		colourSP.fetchSample(colourSample, 0);
		return colourSample; // return array of 3 colours
	}

	public boolean isMoving() {
		return pilot.isMoving();
	}

	public void goUp() {
		pilot.travel(10);
	}

	public void goDown() {
		pilot.rotate(180);
	}

	public void goLeft() {
		pilot.rotate(90);
	}

	public void goRight() {
		pilot.rotate(-90);
	}

	public void goEnter() {
		pilot.travel(20);
	}

	public void controlByKeys() {
		SimpleRobot myRobot = new SimpleRobot();
		int keyPressed = Button.getButtons();
		while (Button.getButtons() != Keys.ID_ESCAPE) {
			switch (keyPressed) {
			case Keys.ID_UP:
				myRobot.goUp();
				break;
			case Keys.ID_LEFT:
				myRobot.goLeft();
				break;
			case Keys.ID_RIGHT:
				myRobot.goRight();
				break;
			case Keys.ID_ENTER:
				myRobot.goEnter();
				break;
			case Keys.ID_DOWN:
				myRobot.goDown();
				break;
			default:
				break;
			}
			Button.waitForAnyPress();
			keyPressed = Button.getButtons();
		}
	}
	
	public void ultrasonicTest() {
		uSensorMotor = new EV3LargeRegulatedMotor(myEV3.getPort("A"));
		while (Button.getButtons() != Keys.ID_ESCAPE) {
			uSensorMotor.forward();
		}
	}
	
	public void irTest() {
		irDistSP = irSensor.getDistanceMode();
		while (Button.getButtons() != Keys.ID_ESCAPE) {
			irDistSP.fetchSample(irDistSample, 0);
			if (irDistSample[0] < 15) {
				pilot.travel(-20, true);
			}
		}
	}
	
	public static void main(String[] args) {
		SimpleRobot myRobot = new SimpleRobot();
		myRobot.ultrasonicTest();
	}

}
