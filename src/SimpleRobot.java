	
import java.awt.List;
import java.util.function.DoubleToLongFunction;

import javax.xml.crypto.dsig.XMLSignature.SignatureValue;

import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.sensor.EV3ColorSensor;
import lejos.hardware.sensor.EV3IRSensor;
import lejos.hardware.sensor.EV3TouchSensor;
import lejos.hardware.sensor.EV3UltrasonicSensor;
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
	private OccupanyGridMap gridMap;
	private GraphicsLCD lcd;
	private EV3Server server;
	private double rightDist, frontDist, leftDist;
	public int robot_h = 0;
	public int robot_w = 0;

	// need to be calibrated
	private static final double WIDTH = 148;

	// need to be calibrated
	private static final double HEIGHT = 193;
	private static final int H_GRID = 6;
	private static final int W_GRID = 6;
	private static final double W_MOVE = WIDTH / W_GRID;
	private static final double H_MOVE = HEIGHT / H_GRID;
	private OdometryPoseProvider poseProvider;
	
	private boolean canCalibrate = false;
	private double lastRight = -1;

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
		if (Float.isInfinite(distance)){
			distance = 100;
		}
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
		Wheel leftWheel = WheeledChassis.modelWheel(Motor.B, 3.3).offset(-10.6);
		Wheel rightWheel = WheeledChassis.modelWheel(Motor.C, 3.3).offset(10.6);
		Chassis myChassis = new WheeledChassis(new Wheel[] { leftWheel, rightWheel }, WheeledChassis.TYPE_DIFFERENTIAL);
		pilot = new MovePilot(myChassis);
	}

	private void setupGridMap() {
		gridMap = new OccupanyGridMap(H_GRID, W_GRID, H_MOVE, W_MOVE);
	}

	private void setupBehaviours() {
		moveBehaviour = new MoveBehaviour(this);
		backBehaviour = new BackBehaviour(this, pilot);
		Behavior[] behaviors = { moveBehaviour, backBehaviour};
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

	/*
	public int[] poseToGrid(float h, float w) {
		int h_grid_num = (int) (h / H_MOVE);
		int w_grid_num = (int) (w / W_MOVE);
		return new int[] { h_grid_num, w_grid_num };
	}*/

	public void forward() {
		getInfo();
		if ((getHeading() % 90 == 0 && rightDist > H_MOVE) || (getHeading() % 180 == 0 && rightDist > W_MOVE)) {
			pilot.rotate(90);
			if (getHeading() % 180 == 0) {
				move(H_MOVE);
			} else {
				move(W_MOVE);
			}
		} else if ((getHeading() % 180 == 0 && frontDist < H_MOVE) || (getHeading() % 90 == 0 && frontDist < W_MOVE)) {
			pilot.rotate(-90);
		} else if (getHeading() % 180 == 0) {
			//System.out.println("MOVE HHHHHHHHHHH");
			if (robot_w == 0 || robot_w == W_GRID - 1){
				canCalibrate = true;
				Sound.beep();
			}
			move(H_MOVE);
		} else {
			if (robot_h == 0 || robot_h == H_GRID - 1) {
				canCalibrate = true;
				Sound.beep();
			}
			//System.out.println("MOVE WWWWWWWWWWW");
			move(W_MOVE);
		}
	}
	
	public int getHeading(){
		double origin = poseProvider.getPose().getHeading();
		int sign = origin > 0 ? 1: -1;
		origin = Math.abs(origin);
		double remainder = origin % 90;
		int num = (int) origin / 90;
		if (remainder > 80) {
			remainder = 90;
		} else {
			remainder = 0;
		}
		int n = (int)(num*90 + remainder)*sign;
		if (n<=-180) n+= 360;
		if (n>=180) n=360-n;
		return n;
	}
	
	public void processInfo(double distance, int relativeHeading) {
		int sensorHeading = (int) (relativeHeading - getHeading());
		if (sensorHeading <= -180) sensorHeading += 360;
		if (sensorHeading >= 180) sensorHeading = 360 - sensorHeading;
		//System.out.println("Transferred sensorHeading: " + sensorHeading);
		int[] obstacle_grid;
		int[] robot_grid = new int[]{robot_h,robot_w};
		//System.out.println("Robot_grid: " + robot_grid[0] + "," + robot_grid[1]);
		int h_robot = robot_grid[0];
		int w_robot = robot_grid[1];
		int h_target=robot_grid[0];
		int w_target=robot_grid[1];
		if (sensorHeading == 0) {
			h_target += 1 + distance / H_MOVE;
			for (int h=h_robot+1; h<h_target && h <=H_GRID-1 && h>=0; h++){
				gridMap.update(h, w_robot, false);
				//server.send("Going to update obstacle in the front: " + h + "," + w_robot);
			}
		} else if (sensorHeading == 180) {
			h_target -= 1 + distance / H_MOVE;
			for (int h=h_target+1; h<h_robot && h <= H_GRID-1 && h>=0; h++){
				gridMap.update(h, w_robot, false);
			}
		} else if (sensorHeading == 90) {
			w_target += 1 + distance / W_MOVE;
			for (int w=w_robot+1; w<w_target && w <= W_GRID-1 && w>=0; w++){
				gridMap.update(h_robot,w, false);
			}
 		} else if (sensorHeading == -90) {
 			w_target -= 1 + distance / W_MOVE;
			for (int w=w_target+1; w<w_target && w <= W_GRID-1 && w>=0; w++){
				gridMap.update(h_robot,w, false);
			}
 		} else {
 			for (int i=0; i<=9; i++){
 				//System.out.println("OUT OF RANGE: " + sensorHeading);
 			}
 		}
		/*
		switch (sensorHeading) {
			case 0:
				h_target += 1 + distance / H_MOVE;
				for (int h=h_robot+1; h<h_target && h <=H_GRID-1 && h>=0; h++){
					gridMap.update(h, w_robot, false);
					server.send("Going to update: " + h + "," + w_robot);
				}
				break;
			case 180:
				h_target -= 1 + distance / H_MOVE;
				for (int h=h_target+1; h<h_robot && h <= H_GRID-1 && h>=0; h++){
					gridMap.update(h, w_robot, false);
					server.send("Going to update: " + h + "," + w_robot);
				}
				break;
			case 90:
				w_target += 1 + distance / W_MOVE;
				for (int w=w_robot+1; w<w_target && w <= W_GRID-1 && w>=0; w++){
					gridMap.update(h_robot,w, false);
					//server.send("Going to update: " + h_robot + "," + w);
				}
				break;
			case -90:
				server.send("distance: " + distance + ", W_MOVE: " + W_MOVE);
				w_target -= 1 + distance / W_MOVE;
				server.send("w_target: " + w_target);
				for (int w=w_target+1; w<w_robot && w<=W_GRID-1 && w>=0; w++){
					gridMap.update(h_robot,w, false);
					//server.send("Going to update: " + h_robot + "," + w);
				}
				break;
			default:
				System.out.println("ABSOLUTE ANGLE OUT OF RANGE");
				break;
		}*/
		obstacle_grid = new int[]{h_target, w_target};
		//System.out.println("Obstacle Grid: " + obstacle_grid[0] + "," + obstacle_grid[1]);
		//server.send("Obstacle Grid: " + obstacle_grid[0] + "," + obstacle_grid[1]);
		if (h_target <= H_GRID-1 && w_target <= W_GRID-1 && h_target >= 0 && w_target >= 0) {
			//server.send("Going to update: " + h_target + "," + w_target);
			gridMap.update(h_target,w_target, true);

		}
	}
	
	public void move(double distance) {
		//System.out.println("Now moving ~~~~~~~~~~~~~~~~~~~~~~~~");
		pilot.travel(distance);
		int heading = getHeading();
		switch(heading){
		case 0:
			robot_h += 1;
			break;
		case 180:
			robot_h -= 1;
			break;
		case -90:
			robot_w += 1;
			break;
		case 90:
			robot_w -= 1;
			break;
		default:
			System.out.println("FFFFFFFFFFFFFFFFFFFFFF");
			break;
		}
	}
	
	public void getInfo() {
		float color = getColor();
		int[] robotGrid = new int[]{robot_h,robot_w};
		//System.out.println("Robot: " + robotGrid[0] + "," + robotGrid[1]);
		gridMap.updatePassed(robotGrid[0], robotGrid[1]);
		if (color < 0.05) {
			Sound.beep();
			if (gridMap.isMapFinished() == true) {
				stop();
			} else {
				gridMap.updateEndPoint(robotGrid);
			}
			Sound.beep();
		}
		uSensorMotor.rotate(90);
		leftDist = getSingleDistance();
		server.send("Left: " + leftDist);
		processInfo(leftDist, 90);
		uSensorMotor.rotate(-90);
		frontDist = getSingleDistance();
		server.send("Front: " + frontDist);
		processInfo(frontDist, 0);
		uSensorMotor.rotate(-90);
		double lastRight = rightDist;
		rightDist = getSingleDistance();
		server.send("Right: " + rightDist);
		processInfo(rightDist, -90);
		if (canCalibrate == true) {
			canCalibrate = false;
			double theta;
			double sintheta;
			double p;
			if (getHeading() % 180 == 0) {
				p = (rightDist - lastRight)/H_MOVE;			
			} else {
				p = (rightDist - lastRight)/W_MOVE;
			}
			sintheta = (-1 + Math.sqrt(1+4*Math.pow(p, 2)))/(2*p);
			theta = Math.asin(sintheta)*180/Math.PI;
			pilot.rotate(theta);
		}
		uSensorMotor.rotate(90);
		//frontDist = getSingleDistance();
		//processInfo(frontDist, 0);
		drawMap();
	}

	public void back() {
		pilot.travel(-10);
	}

	public void stop() {
		System.out.println("STOP");
		System.exit(0);
	}

	public void drawMap() {
		System.out.println("\n\n\n\n\n\n");
		lcd.clear();
		int bias = 5;
		int len = 16;
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
					lcd.drawChar('?', bias+len*w+3, bias+len*h+2, GraphicsLCD.VCENTER);
				} else if (h==robot_h && w==robot_w) {
					lcd.drawChar('X', bias+len*w+3, bias+len*h+2, GraphicsLCD.VCENTER);
				} 
			}
		}
		if (gridMap.isEndGridFound()){
			int[] endGrid = gridMap.getEndPoint();
			lcd.drawChar('O', bias+len*endGrid[0]+3, bias+len*endGrid[1]+2, GraphicsLCD.VCENTER);
		}
		//Button.waitForAnyPress();
	}

	// robot: 90 (right) sensorMotor: 90 (left)
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
	
	public void moveTo(List path){
		
	}
	
	public static void main(String[] args) {
		SimpleRobot myRobot = new SimpleRobot();
		myRobot.arbitrator.go();
		myRobot.getInfo();	
	}
}
