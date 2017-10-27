package main;
import java.util.LinkedList;
import behaviors.BackBehavior;
import behaviors.MoveBehavior;
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
import threads.DrawingThread;
import threads.StoppingThread;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.Sound;
import lejos.hardware.lcd.GraphicsLCD;

public class SmartRobot {
	private Brick ev3;
	private EV3TouchSensor leftBump, rightBump;
	private EV3IRSensor irSensor;
	private EV3UltrasonicSensor uSensor;
	private EV3ColorSensor cSensor;
	private SampleProvider leftBumpSP, rightBumpSP, colourSP, ultrasonicDistSP;
	private float[] leftBumpSample, rightBumpSample, colourSample, ultrasonicDistSample;
	private Arbitrator arbitrator;
	private MovePilot pilot;
	private NXTRegulatedMotor uSensorMotor;
	private MoveBehavior moveBehavior;
	private BackBehavior backBehavior;
	private OccupanyGridMap map;
	private GraphicsLCD lcd;
	private double rightDistance, frontDistance, leftDistance;
	private OdometryPoseProvider poseProvider;
	private boolean needCalibrating = false;
	private boolean taskFinished = false;
	private boolean readyToEnd = false;
	// keep track of the current location of the robot
	private int robotH = 0;
	private int robotW = 0;
	// constant variables
	private static final double HEIGHT_OF_ARENA = 193;
	private static final double WIDTH_OF_ARENA = 150;
	// number of grids
	private static final int H_GRID = 6;
	private static final int W_GRID = 6;
	// distance of each movement
	private static final double H_MOVE = HEIGHT_OF_ARENA / H_GRID;
	private static final double W_MOVE = WIDTH_OF_ARENA / W_GRID;
	private static final double RESERVED_DIST_H = 11;
	private static final double RESERVED_DIST_RIGHT = 12;
	// offset and diameter
	private static final double DIAMETER = 3.3;
	private static final double OFFSET = 9.4;
	private static final double ANGULAR_SPEED = 30;
	private static final double BLUE_COLOR_THRESHOLD = 0.05;
	// set up ultrasonic sensor
	private void setupUltrasonicSensor() {
		uSensorMotor = Motor.A;
		uSensor = new EV3UltrasonicSensor(ev3.getPort("S3"));
		ultrasonicDistSP = uSensor.getDistanceMode();
		ultrasonicDistSample = new float[ultrasonicDistSP.sampleSize()];
	}

	// get the distance of the nearest object
	private float getSingleDistance() {
		ultrasonicDistSP.fetchSample(ultrasonicDistSample, 0);
		float dist1 = ultrasonicDistSample[0] * 100;
		ultrasonicDistSP.fetchSample(ultrasonicDistSample, 0);
		float dist2 = ultrasonicDistSample[0] * 100;
		ultrasonicDistSP.fetchSample(ultrasonicDistSample, 0);
		float dist3 = ultrasonicDistSample[0] * 100;
		// get the average of three measure results
		float distance = (dist1 + dist2 + dist3) / 3;
		if (Float.isInfinite(distance)){
			// simply use a very large number if the result is inifinite
			distance = 1000;
		}
		return distance;
	}

	// set up the color sensor -> using RED mode
	private void setupColorSensor() {
		cSensor = new EV3ColorSensor(ev3.getPort("S4"));
		colourSP = cSensor.getRedMode();
		colourSample = new float[colourSP.sampleSize()];
	}
	
	// get the color
	private float getColor() {
		if (cSensor == null) {
			setupColorSensor();
		}
		colourSP.fetchSample(colourSample, 0);
		return colourSample[0];
	}

	// setup touch sensor (maybe useless in this assignment)
	private void setupTouchSensor() {
		leftBump = new EV3TouchSensor(ev3.getPort("S2"));
		rightBump = new EV3TouchSensor(ev3.getPort("S1"));
		leftBumpSP = leftBump.getTouchMode();
		rightBumpSP = rightBump.getTouchMode();
		leftBumpSample = new float[leftBumpSP.sampleSize()];
		rightBumpSample = new float[rightBumpSP.sampleSize()];
	}

	// set up the pilot
	private void setupPilot() {
		Wheel leftWheel = WheeledChassis.modelWheel(Motor.B, DIAMETER).offset(-OFFSET);
		Wheel rightWheel = WheeledChassis.modelWheel(Motor.C, DIAMETER).offset(OFFSET);
		Chassis myChassis = new WheeledChassis(new Wheel[] { leftWheel, rightWheel }, WheeledChassis.TYPE_DIFFERENTIAL);
		pilot = new MovePilot(myChassis);
		pilot.setAngularSpeed(ANGULAR_SPEED);
	}

	// create a new occupancy grid map
	private void setupGridMap() {
		map = new OccupanyGridMap(H_GRID, W_GRID);
	}

	// set up the behaviors
	private void setupBehaviors() {
		moveBehavior = new MoveBehavior(this);
		backBehavior = new BackBehavior(this, pilot);
		Behavior[] behaviors = { moveBehavior, backBehavior};
		arbitrator = new Arbitrator(behaviors, false);
		lcd.clear();
	}

	// set up the position provider (only the heading will be used)
	private void setupPoseProvider() {
		poseProvider = new OdometryPoseProvider(pilot);
		poseProvider.setPose(new Pose(0, 0, 0));
	}

	// close all the sensors and the sensor
	private void closeRobot() {
		leftBump.close();
		rightBump.close();
		irSensor.close();
		cSensor.close();
		uSensor.close();
	}

	// whether the left bumper is pressed
	public boolean isLeftBumpPressed() {
		leftBumpSP.fetchSample(leftBumpSample, 0);
		return (leftBumpSample[0] == 1.0);
	}

	// whether the right bumper is pressed
	public boolean isRightBumpPressed() {
		rightBumpSP.fetchSample(rightBumpSample, 0);
		return (rightBumpSample[0] == 1.0);
	}

	// get the current heading of the robot using the data from position provider
	// change whatever the heading into one of 0,90,-90,180
	private int getHeading(){
		double rawHeading = poseProvider.getPose().getHeading();
		double remainder = Math.abs(rawHeading)%90;
		int number = (int)Math.abs(rawHeading)/90;
		if (remainder > 80) {
			remainder = 90;
		} else {
			remainder = 0;
		}
		int n = (int)(number*90 + remainder)*(rawHeading>0?1:-1);
		if (n<=-180) n+= 360;
		if (n>=180) n=360-n;
		return n;
	}
	
	// whether the robot is at a corner
	private boolean robotIsAtCorner(){
		return (robotH==0||robotH==H_GRID-1)&&(robotW==0||robotW==W_GRID-1);
	}
	
	// move a specific distance and keep track of the grid the robot is currently in
	public void move(double distance) {
		pilot.travel(distance);
		int heading = getHeading();
		switch(heading){
		case 0:
			robotH += 1;
			break;
		case 180:
			robotH -= 1;
			break;
		case -90:
			robotW += 1;
			break;
		case 90:
			robotW -= 1;
			break;
		default:
			break;
		}
	}
	
	// return whether task is finished
	public boolean forward() {
		if (taskFinished) return true;
		if (readyToEnd) {
			if (map.isEndGridFound()==true){
				navigateToGrid(map.getEndPoint());
			} else {
				findEndPoint();
			}
		} else {
			buildMap();
		}
		return false;
	}
	
	// explore the arena and build the map
	private void buildMap(){
		getInformation();
		int heading = getHeading();
		if ((heading % 180 != 0 && rightDistance > H_MOVE) || (heading % 180 == 0 && rightDistance > W_MOVE)) {
			pilot.rotate(90);
			if (getHeading() % 180 == 0) {
				move(H_MOVE);
			} else {
				move(W_MOVE);
			}
			needCalibrating = false;
		} else if ((heading%180==0 && frontDistance<H_MOVE) || (heading%180!=0 && frontDistance<W_MOVE)) {
			System.out.println("Front: "+frontDistance);
			pilot.travel(frontDistance-RESERVED_DIST_H);
			pilot.rotate(-90);
			// pilot.travel(-2);
			needCalibrating = false;
		} else if (heading % 180 == 0) {
			if (robotW == 0 || robotW == W_GRID - 1){
				needCalibrating = true;
			}
			move(H_MOVE);
		} else {
			if (robotH == 0 || robotH == H_GRID - 1) {
				needCalibrating = true;
			}
			move(W_MOVE);
		}
		// after returning to the start point, start to find the end point
		if (map.isMapFinished() && robotH==0 && robotW==0){
			readyToEnd = true;
		}
	}
	
	// process the information (raw sensor data)
	private void processInformation(double distance, int relativeHeading) {
		int sensorHeading = (int) (relativeHeading - getHeading());
		if (sensorHeading <= -180) sensorHeading += 360;
		if (sensorHeading >= 180) sensorHeading = 360 - sensorHeading;
		int targetH=robotH, targetW=robotW;
		if (sensorHeading == 0) {
			targetH += 1 + distance / H_MOVE;
			for (int h=robotH+1; h<targetH && h<=H_GRID-1 && h>=0; h++){
				map.update(h, targetW, false);
			}
		} else if (sensorHeading == 180) {
			targetH -= 1 + distance / H_MOVE;
			for (int h=targetH+1; h<robotH && h<=H_GRID-1 && h>=0; h++){
				map.update(h, targetW, false);
			}
		} else if (sensorHeading == 90) {
			targetW += 1 + distance / W_MOVE;
			for (int w=robotW+1; w<targetW && w<=W_GRID-1 && w>=0; w++){
				map.update(targetH, w, false);
			}
 		} else if (sensorHeading == -90) {
 			targetW -= 1 + distance / W_MOVE;
			for (int w=targetW+1; w<robotW && w<=W_GRID-1 && w>=0; w++){
				map.update(targetH, w, false);
			}
 		} 
		if (targetH<=H_GRID-1 && targetW<=W_GRID-1 && targetH>= 0 && targetW>= 0) {
			map.update(targetH,targetW, true);
		}
	}
	
	//double tantheta;
	//tantheta = p;
	//theta = Math.atan(tantheta)*180/Math.PI
	
	// get information from the environment using sensors
	private void getInformation() {
		// if blue is detected
		float colorDetected = getColor();
		if (robotH<=H_GRID-1&&robotH>=0&&robotW<=W_GRID-1&&robotW>=0) map.updatePassed(robotH, robotW);
		if (!readyToEnd) {
			if (colorDetected < BLUE_COLOR_THRESHOLD) map.updateEndPoint(new int[]{robotH,robotW});
			uSensorMotor.rotate(90);
			leftDistance = getSingleDistance();
			processInformation(leftDistance, 90);
			uSensorMotor.rotate(-90);
			frontDistance = getSingleDistance();
			processInformation(frontDistance, 0);
			uSensorMotor.rotate(-90);
			double lastRightDist = rightDistance;
			rightDistance = getSingleDistance();
			processInformation(rightDistance, -90);
			if (needCalibrating == true) {
				needCalibrating = false;
				calibrate(lastRightDist, rightDistance);
			}
			uSensorMotor.rotate(90);
			frontDistance = getSingleDistance();
			processInformation(frontDistance, 0);
		} else if (colorDetected < BLUE_COLOR_THRESHOLD) {
			Sound.twoBeeps();
			stop();
		}
	}

	// do some calibration
	private void calibrate(double lastRightDist, double rightDist){
		double theta, sintheta, distDiff;
		if (getHeading() % 180 == 0) {
			distDiff = (rightDistance-lastRightDist)/H_MOVE;			
		} else {
			distDiff = (rightDistance-lastRightDist)/W_MOVE;
		}
		sintheta = (-1 + Math.sqrt(1+4*Math.pow(distDiff, 2)))/(2*distDiff);
		if (sintheta>=0.5) sintheta = (-1 - Math.sqrt(1+4*Math.pow(distDiff, 2)))/(2*distDiff);
		theta = Math.asin(sintheta)*180/Math.PI;
		Pose oldPose = poseProvider.getPose();
		Pose copyPose = new Pose(oldPose.getX(), oldPose.getY(), oldPose.getHeading());
		pilot.rotate(theta);
		if(!robotIsAtCorner()){
			if (getHeading()%180==0){
				sintheta=(RESERVED_DIST_RIGHT-rightDistance)/H_MOVE;
			} else{
				sintheta=(RESERVED_DIST_RIGHT-rightDistance)/W_MOVE;
			}
			System.out.println(sintheta);
			theta=Math.asin(sintheta)*180/Math.PI;
			System.out.println(theta);
			Button.waitForAnyPress();
			drawMap();
			pilot.rotate(-theta);
		}
		poseProvider.setPose(copyPose);
	}
	
	// stop the robot 
	public void stop() {
		System.exit(0);
		System.out.println("STOP");
		arbitrator.stop();
		taskFinished = true;
	}

	// draw the map on the LCD screen
	public void drawMap() {
		System.out.println("\n\n\n\n\n\n");
		lcd.clear();
		int bias = 5;
		int len = 15;
		for (int h=0; h<=H_GRID-1; h++) {
			for (int w=0; w<=W_GRID-1; w++) {
				lcd.drawRect(bias+len*w, bias+len*h, len, len);
				if (map.isOccupied(H_GRID-1-h, W_GRID-1-w) == 1) {
					lcd.fillRect(bias+len*w, bias+len*h, len, len);
				} else if (map.isOccupied(H_GRID-1-h, W_GRID-1-w) == -1) {
					lcd.drawChar('?', bias+len*w+3, bias+len*h+2, GraphicsLCD.VCENTER);
				} else if (h==H_GRID-1-robotH && w==W_GRID-1-robotW) {
					lcd.drawChar('X', bias+len*w+3, bias+len*h+2, GraphicsLCD.VCENTER);
				} 
			}
		}
		if (map.isEndGridFound()){
			int[] endGrid = map.getEndPoint();
			lcd.drawChar('O', bias+len*endGrid[0]+3, bias+len*endGrid[1]+2, GraphicsLCD.VCENTER);
		}
	}

	// move to a grid following a path that is found by A* search
	public void navigateToGrid(int[] goal){
		LinkedList<int[]> path = map.aStarPathFinding(new int[]{robotH,robotW}, goal);
		for(int i=0; i<=path.size()-1;i++){
			moveToGrid(path.get(i));
		}
	}
	
	// move to a grid that next to the robot
	private void moveToGrid(int[] goal){
		int heading = getHeading();
		if (goal[0] == robotH+1) {
			// go up
			if (heading == 0){
				move(H_MOVE);
			} else if (heading == 180) {
				move(-H_MOVE);
			} else {
				pilot.rotate(-heading);
				move(H_MOVE);
			}
		} else if (goal[0] == robotH-1){
			// go down
			if (heading == 0){
				move(-H_MOVE);
			} else if (heading == 180) {
				move(H_MOVE);
			} else {
				pilot.rotate(heading);
				move(H_MOVE);
			}
		} else if (goal[1] == robotW+1) {
			// go left
			if (heading == 90) {
				move(-W_MOVE);
			} else if (heading == -90) {
				move(W_MOVE);
			} else if (heading == 0) {
				pilot.rotate(-90);
				move(W_MOVE);
			} else if (heading == 180) {
				pilot.rotate(90);
				move(W_MOVE);
			}
		} else if (goal[1] == robotW-1) {
			// go right
			if (heading == 90) {
				move(W_MOVE);
			} else if (heading == -90) {
				move(-W_MOVE);
			} else if (heading == 0) {
				pilot.rotate(90);
				move(W_MOVE);
			} else if (heading == 180) {
				pilot.rotate(-90);
				move(W_MOVE);
			}
		}
		getInformation();
	}
	
	// follow a path until the task is finished
	public void followPath(LinkedList<int[]> thePath){
		for (int[] grid: thePath){
			moveToGrid(grid);
			if (taskFinished) {
				return;
			}
		}
	}
	
	// keep finding the end point until the task is finished
	public void findEndPoint(){
		while(!taskFinished){
			LinkedList<int[]> bestExplorationPath = map.findTheBestPathToExplore(new int[]{robotH,robotW});
			followPath(bestExplorationPath);
		}
	}
	
	// constructor of SmartRobot
	public SmartRobot() {
		ev3 = BrickFinder.getDefault();
		lcd = ev3.getGraphicsLCD();
		setupPilot();
		setupTouchSensor();
		setupColorSensor();
		setupUltrasonicSensor();
		setupGridMap();
		setupPoseProvider();
		setupBehaviors();
		new StoppingThread(this).start();
		//new DrawingThread(this,20).start();
		//pilot.rotate(90);
		arbitrator.go();
	}
	
	public static void main(String[] args) {
		SmartRobot myRobot = new SmartRobot();
		myRobot.navigateToGrid(new int[]{0,5});
		myRobot.closeRobot();
	}
}
