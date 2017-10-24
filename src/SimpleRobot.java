
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
import lejos.robotics.localization.OdometryPoseProvider;
import lejos.robotics.navigation.MovePilot;
import lejos.robotics.navigation.Pose;
import lejos.robotics.subsumption.Arbitrator;
import lejos.robotics.subsumption.Behavior;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.Keys;
import lejos.hardware.Sound;
import lejos.hardware.lcd.GraphicsLCD;

// to do: how to read local configuration file
// to do: how to print things in a format on EV3' Screen
// to do: how to detect colors

public class SimpleRobot {
	private Brick ev3;
	private EV3TouchSensor leftBump, rightBump;
	private EV3IRSensor irSensor;
	private EV3UltrasonicSensor uSensor;
	private EV3ColorSensor cSensor;
	private SampleProvider leftBumpSP, rightBumpSP, colourSP, ultrasonicDistSP;
	private float[] leftBumpSample, rightBumpSample, colourSample, ultrasonicDistSample;
	private Arbitrator arbitrator;
	public MovePilot pilot;
	private NXTRegulatedMotor uSensorMotor;
	private MoveBehaviour moveBehaviour;
	private BackBehaviour backBehaviour;
	private AvoidBehaviour avoidBehaviour;
	private OccupanyGridMap gridMap;
	private GraphicsLCD lcd;
	private EV3Server server;
	private double rightDist, frontDist, leftDist;
	private final static double DIST = 20;
	private int[] end_grid = new int[]{0,0};

	// need to be calibrated
	private static final double WIDTH = 150;

	// need to be calibrated
	private static final double HEIGHT = 170;;
	private static final int H_GRID = 5;
	private static final int W_GRID = 4;
	private static final double W_MOVE = WIDTH / W_GRID;
	private static final double H_MOVE = HEIGHT / H_GRID;
	private OdometryPoseProvider poseProvider;

	private void setupUltrasonicSensor() {
		uSensorMotor = Motor.A;
		uSensor = new EV3UltrasonicSensor(ev3.getPort("S3"));
		switchUToSingleMode();
	}

	// be able to detect the nearest object
	public void switchUToSingleMode() {
		ultrasonicDistSP = uSensor.getDistanceMode();
		ultrasonicDistSample = new float[ultrasonicDistSP.sampleSize()];
	}

	// get the distance of the nearest object
	public float getSingleDistance() {
		if (uSensor.getCurrentMode() != 0) {
			switchUToSingleMode();
		}
		ultrasonicDistSP.fetchSample(ultrasonicDistSample, 0);
		float dist1 = ultrasonicDistSample[0] * 100;
		ultrasonicDistSP.fetchSample(ultrasonicDistSample, 0);
		float dist2 = ultrasonicDistSample[0] * 100;
		ultrasonicDistSP.fetchSample(ultrasonicDistSample, 0);
		float dist3 = ultrasonicDistSample[0] * 100;
		float distance = (dist1 + dist2 + dist3) / 3;
		return distance;
	}

	public float getColor() {
		if (cSensor == null) {
			setupColorSensor();
		}
		colourSP.fetchSample(colourSample, 0);
		return colourSample[0];
	}

	private void setupColorSensor() {
		cSensor = new EV3ColorSensor(ev3.getPort("S4"));
		colourSP = cSensor.getRedMode();
		colourSample = new float[colourSP.sampleSize()];
	}

	private void setupTouchSensor() {
		leftBump = new EV3TouchSensor(ev3.getPort("S2"));
		rightBump = new EV3TouchSensor(ev3.getPort("S1"));
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

	private void setupGridMap() {
		gridMap = new OccupanyGridMap(H_GRID, W_GRID);
	}

	private void setupBehaviours() {
		moveBehaviour = new MoveBehaviour(this);
		backBehaviour = new BackBehaviour(this, pilot);
		avoidBehaviour = new AvoidBehaviour(this, pilot);
		Behavior[] behaviors = { moveBehaviour, avoidBehaviour, backBehaviour};
		arbitrator = new Arbitrator(behaviors, false);
		lcd.clear();
	}

	private void setupPoseProvider() {
		poseProvider = new OdometryPoseProvider(pilot);
		poseProvider.setPose(new Pose(10, 10, 0));
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

	public void ultrasonicTest() {
		uSensorMotor.rotate(90, true);
		uSensorMotor.rotate(-180, true);
		uSensorMotor.rotate(90, true);
	}

	public void serverTest() {
		EV3Server server = new EV3Server(this, 2000);
		Thread serverThread = new Thread(server);
		serverThread.start();
	}

	public void colorDetectTest() {
		while (Button.getButtons() != Keys.ID_ESCAPE) {
			pilot.travel(20, true);
			float color = getColor();
			if (color < 0.02) {
				pilot.travel(-10);
			}
		}
	}

	public void beepTest() {
		while (Button.getButtons() != Keys.ID_ESCAPE) {
			Sound.beep();
		}
	}

	public void moveH() {
		pilot.travel(H_MOVE);
	}

	public void moveW() {
		pilot.travel(W_MOVE);
	}

	public void moveAroundWall() {
		// move forward

		// turn left
		// move forward
		// turn left
		// move forward
		// turn left
		// move forward
	}

	public int[] poseToGrid(float h, float w) {
		int h_grid_num = (int) (h / H_MOVE);
		int w_grid_num = (int) (w / W_MOVE);
		return new int[] { h_grid_num, w_grid_num };
	}

	public void forward() {
		getInfo();
		if (rightDist > DIST) {
			pilot.rotate(-90);
			if (poseProvider.getPose().getHeading() % 180 == 0) {
				pilot.travel(W_MOVE);
			} else {
				pilot.travel(H_MOVE);
			}
		} else if (frontDist < DIST) {
			pilot.rotate(90);
		} else if (poseProvider.getPose().getHeading() % 180 == 0) {
			pilot.travel(H_MOVE);
		} else {
			pilot.travel(W_MOVE);
		}
	}

	public void processInfo(double distance, int relativeHeading) {
		Pose pose = poseProvider.getPose();
		int sensorHeading = (int) (relativeHeading + pose.getHeading());
		if (sensorHeading <= -180) sensorHeading += 360;
		if (sensorHeading >= 180) sensorHeading = 360 - sensorHeading;
		System.out.println("Transferred sensorHeading: " + sensorHeading);
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		int[] obstacle_grid;
		int[] robot_grid = poseToGrid(pose.getX(), pose.getY());
		gridMap.update(robot_grid, false);
		System.out.println("Robot_grid: " + robot_grid[0] + "," + robot_grid[1]);
		try {
			Thread.sleep(1000);
		} catch (InterruptedException e) {
			e.printStackTrace();
		}
		int h_robot = robot_grid[0];
		int w_robot = robot_grid[1];
		int h_target=robot_grid[0];
		int w_target=robot_grid[1];
		switch (sensorHeading) {
			case 0:
				h_target += 1 + distance / H_MOVE;
				for (int h=h_robot+1; h<h_target && h <=H_GRID-1 && h>=0; h++){
					gridMap.update(new int[]{h, w_robot}, false);
					server.send("Going to update: " + h + "," + w_robot);
				}
				break;
			case 180:
				h_target -= 1 + distance / H_MOVE;
				for (int h=h_target+1; h<h_robot && h <= H_GRID-1 && h>=0; h++){
					gridMap.update(new int[]{h, w_robot}, false);
					server.send("Going to update: " + h + "," + w_robot);
				}
				break;
			case 90:
				w_target += 1 + distance / W_MOVE;
				for (int w=w_robot+1; w<w_target && w <= W_GRID-1 && w>=0; w++){
					gridMap.update(new int[]{h_robot,w}, false);
					//server.send("Going to update: " + h_robot + "," + w);
				}
				break;
			case -90:
				server.send("distance: " + distance + ", W_MOVE: " + W_MOVE);
				w_target -= 1 + distance / W_MOVE;
				server.send("w_target: " + w_target);
				for (int w=w_target+1; w<w_robot && w<=W_GRID-1 && w>=0; w++){
					gridMap.update(new int[]{h_robot,w}, false);
					//server.send("Going to update: " + h_robot + "," + w);
				}
				break;
			default:
				System.out.println("ABSOLUTE ANGLE OUT OF RANGE");
				break;
		}
		obstacle_grid = new int[]{h_target, w_target};
		System.out.println("Obstacle Grid: " + obstacle_grid[0] + "," + obstacle_grid[1]);
		//server.send("Obstacle Grid: " + obstacle_grid[0] + "," + obstacle_grid[1]);
		if (h_target <= H_GRID-1 && w_target <= W_GRID-1 && h_target >= 0 && w_target >= 0) {
			server.send("Going to update: " + h_target + "," + w_target);
			gridMap.update(new int[]{h_target,w_target}, true);

		}
		drawMap();
	}
	
	public void getInfo() {
		float color = getColor();
		if (color < 0.05) {
			end_grid = poseToGrid(poseProvider.getPose().getX(), poseProvider.getPose().getY());
			stop();
		}
		uSensorMotor.rotate(90);
		leftDist = getSingleDistance();
		server.send("Left: " + leftDist);
		Button.waitForAnyPress();
		processInfo(leftDist, 90);
		uSensorMotor.rotate(-90);
		frontDist = getSingleDistance();
		//server.send("Front: " + frontDist);
		processInfo(frontDist, 0);
		uSensorMotor.rotate(-90);
		rightDist = getSingleDistance();
		processInfo(rightDist, -90);
		uSensorMotor.rotate(90);
		frontDist = getSingleDistance();
		processInfo(frontDist, 0);
		drawMap();
	}

	public void back() {
		pilot.travel(-10);
	}

	public void stop() {
		System.out.println("STOP");
		//System.exit(0);
	}

	public void drawMap() {
		System.out.println("\n\n\n\n\n\n");
		lcd.clear();
		int bias = 30;
		int len = 18;
		for (int h = 0; h < H_GRID; h++) {
			for (int w = 0; w < W_GRID; w++) {
				lcd.drawRect(bias+len*w, bias+len*h, len, len);
			}
		}
		for (int h = 0; h < H_GRID; h++) {
			for (int w = 0; w < W_GRID; w++) {
				if (gridMap.isOccupied(h, w) == 1) {
					lcd.fillRect(bias+len*w, bias+len*h, len, len);
				} else if (gridMap.isOccupied(h, w) == -1) {
					lcd.drawChar('?', bias+len*w+3, bias+len*h+2, GraphicsLCD.VCENTER);;
				}
			}
		}
		Button.waitForAnyPress();
		lcd.clear();
	}

	public SimpleRobot() {
		ev3 = BrickFinder.getDefault();
		lcd = ev3.getGraphicsLCD();
		setupPilot();
		setupTouchSensor();
		setupColorSensor();
		setupUltrasonicSensor();
		setupGridMap();
		setupPoseProvider();
		setupBehaviours();
		server = new EV3Server(this, 1000);
		server.start();
	}
	
	public static void main(String[] args) {
		SimpleRobot myRobot = new SimpleRobot();
		myRobot.arbitrator.go();
		myRobot.getInfo();
	}
}
