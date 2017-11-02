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
import threads.ServerThread;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.Sound;
import lejos.hardware.lcd.GraphicsLCD;

// to do
// change distance every grid using ultrasonic sensor
// make get distance method more accurate
// pilot angular speed
// configuration file
                      
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
	private OccupancyGridMap map;
	private GraphicsLCD lcd;
	private double rightDistance, frontDistance, leftDistance;
	private OdometryPoseProvider poseProvider;
	private boolean needCalibrating = false;
	private boolean taskFinished = false;
	private boolean readyToEnd = false;
	private int lastHeading = -1;
	private int step = 0;
	private ServerThread server;
	// keep track of the current location of the robot
	private int robotH = 0;
	private int robotW = 0;
	// constant variables
	private static final double HEIGHT_OF_ARENA = 190;
	private static final double WIDTH_OF_ARENA = 150;
	// number of grids
	private static final int H_GRID = 6;
	private static final int W_GRID = 5;
	// distance of each movement
	private static final double H_MOVE = HEIGHT_OF_ARENA / H_GRID;
	private static final double W_MOVE = WIDTH_OF_ARENA / W_GRID;
	private static final double RESERVED_DIST_H = 11;
	private static final double RESERVED_DIST_RIGHT = 13;
	private static final double THIRD_CALIBRATE_MOVE = 9;
	// offset and diameter
	private static final double DIAMETER = 3.3;
	private static final double OFFSET = 10;
	private static final double ANGULAR_SPEED = 100;
	private static final double ANGULAR_ACCELERATION = 200;
	private static final double BLUE_COLOR_THRESHOLD = 0.1;
	private static final int REPEAT_SCAN_TIMES = 6;
	// set up ultrasonic sensor
	private void setupUltrasonicSensor() {
		uSensorMotor = Motor.A;
		uSensor = new EV3UltrasonicSensor(ev3.getPort("S3"));
		ultrasonicDistSP = uSensor.getDistanceMode();
		ultrasonicDistSample = new float[ultrasonicDistSP.sampleSize()];
	}

	// get the distance of the nearest object
	private float getSingleDistance() {
		float validCount = 0;
		float validSum = 0;
		for (int i=0; i<=REPEAT_SCAN_TIMES-1; i++){
			ultrasonicDistSP.fetchSample(ultrasonicDistSample, 0);
			float dist = ultrasonicDistSample[0] * 100;
			if (!Float.isInfinite(dist)) {
				validCount += 1;
				validSum += dist;
			}
		}
		if (validCount == 0){
			uSensorMotor.rotate(-3);
			float upperDist = getSingleDistance();
			uSensorMotor.rotate(6);
			float lowerDist = getSingleDistance();
			uSensorMotor.rotate(-3);
			return (upperDist+lowerDist)/2;
		} else {
			return validSum / validCount;
		}
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
		pilot.setAngularAcceleration(ANGULAR_ACCELERATION);
	}

	// create a new occupancy grid map
	private void setupGridMap() {
	
		map = new OccupancyGridMap(H_GRID, W_GRID);
		/*
		for (int i=0; i<=H_GRID-1; i++){
			for (int j=0; j<=W_GRID-1; j++){
				map.update(i, j, false);
			}
		}
		readyToEnd = true;
		map.update(4, 1, true);
		//map.update(1, 4, true);
		map.updateEndPoint(new int[]{H_GRID-1,W_GRID-1});*/
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
		int number = (int) Math.abs(rawHeading)/90;
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
	private void move(double distance) {
		pilot.travel(distance);
		int heading = getHeading();
		int sign = distance > 0 ? 1 : -1;
		switch(heading){
		case 0:
			robotH += 1 * sign;
			break;
		case 180:
			robotH -= 1 * sign;
			break;
		case -90:
			robotW += 1 * sign;
			break;
		case 90:
			robotW -= 1 * sign;
			break;
		default:
			break;
		}
	}
	
	// return whether task is finished
	public boolean forward() {
		//Button.waitForAnyPress();
		if (taskFinished) {
			return true;
		}
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
		//Button.waitForAnyPress();
		int heading = getHeading();
		lastHeading = heading;
		if ((heading % 180 != 0 && rightDistance > H_MOVE) || (heading % 180 == 0 && rightDistance > W_MOVE)) {
			pilot.rotate(90);
			if (getHeading() % 180 == 0) {
				move(H_MOVE);
			} else {
				move(W_MOVE);
			}
			needCalibrating = false;
		/*} else if ((heading%180==0 && frontDistance<H_MOVE && leftDistance < W_MOVE) || (heading%180!=0 && frontDistance<W_MOVE && leftDistance < H_MOVE)) {
			pilot.rotate(-180);
			needCalibrating = false;
		}*/} else if ((heading%180==0 && frontDistance<H_MOVE) || (heading%180!=0 && frontDistance<W_MOVE)) {
			pilot.travel(frontDistance-RESERVED_DIST_H);
			pilot.rotate(-90);
			needCalibrating = false;
		} else if (heading % 180 == 0) {
			if ((robotW==0 && heading == 0) || (robotW==W_GRID-1 && heading == 180)){
				needCalibrating = true;
			}
			move(H_MOVE);
		} else {
			if ((robotH==0 && heading == 90) || (robotH==H_GRID-1 && heading == -90)) {
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
			targetH += 1 + (int)(distance/H_MOVE);
			server.sendToClient("Detected Grid: ("+targetH+","+targetW+")");
			for (int h=robotH+1; h<targetH && h<=H_GRID-1 && h>=0; h++){
				server.sendToClient("Update false: " + h + "," + targetW + "\n");
				map.update(h, targetW, false);
			}
		} else if (sensorHeading == 180) {
			targetH -= 1 + (int)(distance/H_MOVE);
			server.sendToClient("Detected Grid: ("+targetH+","+targetW+")");
			for (int h=targetH+1; h<robotH && h<=H_GRID-1 && h>=0; h++){
				//Button.waitForAnyPress();
				map.update(h, targetW, false);
			}
		} else if (sensorHeading == 90) {
			targetW += 1 + (int)(distance/W_MOVE);
			server.sendToClient("Detected Grid: ("+targetH+","+targetW+")");
			for (int w=robotW+1; w<targetW && w<=W_GRID-1 && w>=0; w++){
				map.update(targetH, w, false);
			}
 		} else if (sensorHeading == -90) {
 			targetW -= 1 + (int)(distance/W_MOVE);
 			server.sendToClient("Detected Grid: ("+targetH+","+targetW+")");
			for (int w=targetW+1; w<robotW && w<=W_GRID-1 && w>=0; w++){
				map.update(targetH, w, false);
			}
 		} 
		if (targetH<=H_GRID-1 && targetW<=W_GRID-1 && targetH>= 0 && targetW>= 0) {
			map.update(targetH,targetW, true);
		}
	}
	
	// get information from the environment using sensors
	private void getInformation() {
		// if blue is detected
		float colorDetected = getColor();
		if (robotH<=H_GRID-1&&robotH>=0&&robotW<=W_GRID-1&&robotW>=0){
			map.updatePassed(robotH, robotW);
		}
		if (colorDetected < BLUE_COLOR_THRESHOLD){
			Sound.beep();
			map.updateEndPoint(new int[]{robotH,robotW});
			if (readyToEnd){
				taskFinished = true;
				Sound.twoBeeps();
				stop();
				return;
			}	
		}
		// first calibrate (adjust heading)
		uSensorMotor.rotate(-90);
		double lastRightDist = rightDistance;
		rightDistance = getSingleDistance();
		//server.sendToClient("Last: " + lastHeading);
		//server.sendToClient("Right: " + getHeading());
		//server.sendToClient("Last=Right? " + (lastHeading==getHeading()));
		if (lastHeading == getHeading() && rightDistance < W_MOVE) {
			needCalibrating = true;
		} else {
			needCalibrating = false;
		}
		//server.sendToClient("Calirate? " + needCalibrating);
		if (needCalibrating == true) {
			firstCalibrate(lastRightDist, rightDistance);
		}
		// second calibrate (adjust front distance)
		uSensorMotor.rotate(90);
		frontDistance = getSingleDistance();
		secondCalibrate();
		// now hopefully we can get the right information
		// front
		frontDistance = getSingleDistance();
		processInformation(frontDistance, 0);
		// left
		uSensorMotor.rotate(90);
		//Button.waitForAnyPress();
		leftDistance = getSingleDistance();
		processInformation(leftDistance, 90);
		// front again
		uSensorMotor.rotate(-90);
		frontDistance = getSingleDistance();
		processInformation(frontDistance, 0);
		// right
		uSensorMotor.rotate(-90);
		rightDistance = getSingleDistance();
		processInformation(rightDistance, -90);
		server.sendToClient(makeMessage());
		// third calibrate
		/*
		if (needCalibrating == true && !readyToEnd){
			needCalibrating = false;
			thirdCalibrate();
		}
		rightDistance = getSingleDistance();
		*/
		if (needCalibrating == true) {
			thirdCalibrate();
		}
		uSensorMotor.rotate(90);
	}

	private String makeMessage() {
		String message = "======================================================================\n";
		message += "Step " + (++step) + ":\n";
		message += "Current Grid: (" + robotH + "," + robotW + ")\n"; 
		message += "Last Heading: " + lastHeading + "\n";
		message += "Heading: " + getHeading() + "\n";
		message += "front Distance: " + frontDistance + "\n";
		message += "Left Distance: " + leftDistance + "\n";
		message += "Right Distance: " + rightDistance + "\n";
		//Button.waitForAnyPress();
		return message;
	}
	
	// do some calibration
	private void firstCalibrate(double lastRightDist, double rightDist){
		double lastRight, right;
		double theta, tantheta;
		if (getHeading() % 180 == 0) {
			lastRight = lastRightDist%H_MOVE;
			right=rightDist%H_MOVE;
			tantheta = (right-lastRight)/H_MOVE;			
		} else {
			lastRight = lastRightDist%W_MOVE;
			right=rightDist%W_MOVE;
			tantheta = (right-lastRight)/W_MOVE;
		}
		theta = Math.atan(tantheta)*180/Math.PI;
		Pose oldPose = poseProvider.getPose();
		Pose copyPose = new Pose(oldPose.getX(), oldPose.getY(), oldPose.getHeading());
		pilot.rotate(theta);
		/*
		
		*/
		poseProvider.setPose(copyPose);
	}
	
	private void secondCalibrate() {
		double heading = getHeading();
		double remainder;
		if (heading % 180 == 0){
			remainder = frontDistance % H_MOVE;
			pilot.travel(remainder-RESERVED_DIST_H);
		} else {
			remainder = frontDistance % W_MOVE;
			pilot.travel(remainder-RESERVED_DIST_H);
		}
	}
	
	/*
	private void thirdCalibrate() {
		double sintheta, theta;
		if(!robotIsAtCorner()){
			Pose oldPose = poseProvider.getPose();
			Pose copyPose = new Pose(oldPose.getX(), oldPose.getY(), oldPose.getHeading());
			if (getHeading()%180==0){
				sintheta=(RESERVED_DIST_RIGHT-rightDistance)/H_MOVE;
			} else{
				sintheta=(RESERVED_DIST_RIGHT-rightDistance)/W_MOVE;
			}
			lcd.clear();
			if (Math.abs(sintheta)>1){
				Button.waitForAnyPress();
			}
			theta=Math.asin(sintheta)*180/Math.PI;
			pilot.rotate(-4*theta/5);
			poseProvider.setPose(copyPose);
		}
	}*/
	
	private void thirdCalibrate() {
		Sound.beep();
		//Button.waitForAnyPress();
		double dist;
		if (getHeading() % 180 == 0) {
			dist = rightDistance%W_MOVE-RESERVED_DIST_RIGHT;
		} else {
			dist = rightDistance%H_MOVE-RESERVED_DIST_RIGHT;
		}
		double sintheta = dist/THIRD_CALIBRATE_MOVE;
		double theta = Math.asin(sintheta)*180/Math.PI;
		if (theta > 15) theta = 15;
		if (Math.abs(theta) > 8){
			//server.sendToClient("The theta for third calibrate: " + theta);
			pilot.rotate(-theta);
			pilot.travel(-THIRD_CALIBRATE_MOVE);
			pilot.rotate(theta);
			pilot.travel(THIRD_CALIBRATE_MOVE);
		} 
	}
	
	// stop the robot 
	public void stop() {
		System.exit(0);
		//System.out.println("STOP");
		arbitrator.stop();
		taskFinished = true;
	}

	// draw the map on the LCD screen
	public void drawMap() {
		//System.out.println("\n\n\n\n\n\n");
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
				} else if (map.isEndGridFound() && h==H_GRID-1-map.getEndPoint()[0] && w==W_GRID-1-map.getEndPoint()[1]){
					lcd.drawChar('O', bias+len*w+3, bias+len*h+2, GraphicsLCD.VCENTER);
				}
			}
		}
	}

	// move to a grid following a path that is found by A* search
	private void navigateToGrid(int[] goal){
		while(robotH!=goal[0] || robotW!=goal[1]) {
			LinkedList<int[]> path = map.aStarPathFinding(new int[]{robotH,robotW}, goal);
			moveToGrid(path.get(0));
		}
		taskFinished = true;
	}
	
	// move to a grid that next to the robot
	private void moveToGrid(int[] goal){
		int heading = getHeading();
		lastHeading = heading;
		if (goal[0] == robotH+1) {
			// go up
			server.sendToClient("Go Up");
			if (heading == 0){
				move(H_MOVE);
			} else if (heading == 180) {
				move(-H_MOVE);
			} else {
				pilot.rotate(-heading);
				move(H_MOVE);
			}
			
			needCalibrating = true;
		} else if (goal[0] == robotH-1){
			// go down
			server.sendToClient("Go Down");
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
			server.sendToClient("Go Left");
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
			server.sendToClient("Go Right");
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
		needCalibrating = true;
	}
	
	// follow a path until the task is finished
	private void followPath(LinkedList<int[]> thePath){
		for (int[] grid: thePath){
			moveToGrid(grid);
			if (taskFinished) {
				return;
			}
		}
	}
	
	// keep finding the end point until the task is finished
	private void findEndPoint(){
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
		server = new ServerThread();
		server.start();
		new StoppingThread(this).start();
		new DrawingThread(this,200).start();
		arbitrator.go();
		//pilot.rotate(-90);
		Sound.beepSequence();
		Sound.beepSequenceUp();
	}
	
	public static void main(String[] args) {
		SmartRobot myRobot = new SmartRobot();
		myRobot.closeRobot();
	}
}
