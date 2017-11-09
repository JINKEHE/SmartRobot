package main;
import java.text.DecimalFormat;
import java.util.LinkedList;
import behaviors.MoveBehavior;
import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
import lejos.hardware.Button;
import lejos.hardware.Keys;
//import lejos.hardware.Button;
import lejos.hardware.sensor.EV3ColorSensor;
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
//import threads.DrawingThread;
import threads.StoppingThread;
import threads.ServerThread;
import lejos.hardware.motor.Motor;
import lejos.hardware.motor.NXTRegulatedMotor;
import lejos.hardware.Sound;
import lejos.hardware.lcd.GraphicsLCD;
                      
public class SmartRobot {
	private Brick ev3;
	private EV3TouchSensor leftBump, rightBump;
	private EV3UltrasonicSensor uSensor;
	private EV3ColorSensor cSensor;
	private SampleProvider leftBumpSP, rightBumpSP, colourSP, ultrasonicDistSP;
	private float[] leftBumpSample, rightBumpSample, colourSample, ultrasonicDistSample;
	private Arbitrator arbitrator;
	private MovePilot pilot;
	private NXTRegulatedMotor uSensorMotor;
	private MoveBehavior moveBehavior;
	private OccupancyGridMap map;
	private GraphicsLCD lcd;
	private double rightDistance, frontDistance, leftDistance;
	public OdometryPoseProvider poseProvider;
	//private boolean needCalibrating = false;
	private boolean taskFinished = false;
	private boolean readyToEnd = false;
	private int lastHeading = -1;
	private int step = 0;
	public ServerThread server;
	// keep track of the current location of the robot
	private int robotH = 0;
	private int robotW = 0;
	// constant variables
	private static final double HEIGHT_OF_ARENA = 198;
	private static final double WIDTH_OF_ARENA = 153.55;
	// number of grids
	private static final int H_GRID = 6;
	private static final int W_GRID = 5;
	// distance of each movement
	private static final double H_MOVE = HEIGHT_OF_ARENA / H_GRID;
	private static final double W_MOVE = WIDTH_OF_ARENA / W_GRID;
	private static final double RESERVED_DIST_H = 9.5;
	private static final double RESERVED_DIST_W = 12;
	private static final double RESERVED_DIST_RIGHT = 12;
	private static final double THIRD_CALIBRATE_MOVE = 10;
	private static final int SCAN_DELAY = 40;
	// offset and diameter
	private static final double DIAMETER = 3.3;
	private static final double OFFSET = 10;
	private static final double ANGULAR_SPEED = 50;
	private static final double ANGULAR_ACCELERATION = 200;
	private static final float BLUE_COLOR = 0.15f;
	private static final int REPEAT_SCAN_TIMES = 6;
	private StoppingThread stoppingThread;
	private static DecimalFormat df = new DecimalFormat("#.#");
	// set up ultrasonic sensor
	private void setupUltrasonicSensor() {
		uSensorMotor = Motor.A;
		uSensor = new EV3UltrasonicSensor(ev3.getPort("S3"));
		ultrasonicDistSP = uSensor.getDistanceMode();
		ultrasonicDistSample = new float[ultrasonicDistSP.sampleSize()];
	}

	// get a single distance
	// if there's an infinity, move backwards and get the distance again until
	// there's no infinity
	private float getDistanceOnce() {
		try {
			Thread.sleep(SCAN_DELAY);
		} catch (InterruptedException e) {
		}
		ultrasonicDistSP.fetchSample(ultrasonicDistSample, 0);
		float dist = ultrasonicDistSample[0] * 100;
		int c = 1;
		while (Float.isInfinite(dist)) {
			if (c == 1) {
				pilot.travel(-1);
				ultrasonicDistSP.fetchSample(ultrasonicDistSample, 0);
				dist = ultrasonicDistSample[0] * 100;
				c = -1;
			} else if (c == -1) {
				pilot.rotate(1);
				ultrasonicDistSP.fetchSample(ultrasonicDistSample, 0);
				dist = ultrasonicDistSample[0] * 100;
				pilot.rotate(-1);
				c = 1;
			}
		}
		return dist;
	}
	
	// get the distance of the nearest object
	private double getDistance() {
		double threshold = 0.5;
		double average;
		while (true) {
			average = 0;
			float[] dists = new float[REPEAT_SCAN_TIMES];
			for (int i=0; i<=REPEAT_SCAN_TIMES-1; i++) {
				dists[i] = getDistanceOnce();
				//server.sendToClient("\ndist: " + dists[i] + "\n");
				average += dists[i]/REPEAT_SCAN_TIMES;
			}
			for (int i=0; i<=REPEAT_SCAN_TIMES-1; i++) {
				if (Math.abs(average-dists[i])>threshold) {
					continue;
				}
			}
			break;
		}
		return average;
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
		try {
			Thread.sleep(500);
		} catch (InterruptedException e) {
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
		}*/
		/*
		readyToEnd = true;
		map.update(4, 1, true);
		//map.update(1, 4, true);
		map.updateEndPoint(new int[]{H_GRID-1,W_GRID-1});*/
		drawMap();
	}

	// set up the behaviors
	private void setupBehaviors() {
		moveBehavior = new MoveBehavior(this);
		Behavior[] behaviors = { moveBehavior };
		arbitrator = new Arbitrator(behaviors, false);
		System.out.println("\n\n\n\n\n\n");
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
	// not useful anymore
	 
	private boolean robotIsAtCorner(){
		return (robotH==0||robotH==H_GRID-1)&&(robotW==0||robotW==W_GRID-1);
	}
	
	private void move(double distance) {
		int newH = robotH;
		int newW = robotW;
		int heading = getHeading();
		int sign = distance > 0 ? 1 : -1;
		switch(heading){
		case 0:
			newH += 1 * sign;
			break;
		case 180:
			newH -= 1 * sign;
			break;
		case -90:
			newW += 1 * sign;
			break;
		case 90:
			newW -= 1 * sign;
			break;
		default:
			break;
		}
		if (isWithinArena(newH, newW)) {
			robotH = newH;
			robotW = newW;
			pilot.travel(distance);
		} else {
			server.sendToClient("Movement invalid!");
		}
		drawMap();
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
	
	private boolean isWithinArena(int h, int w) {
		if (0 <= h && H_GRID-1 >=h && 0 <= w && W_GRID-1 >=w) {
			return true;
		} else {
			return false;
		}
	}
	
	/*
	private boolean isAtCorner() {
		if ((robotH==0 || robotH==H_GRID-1) && (robotW==0 || robotW==W_GRID-1)) {
			return true;
		} else {
			return false;
		}
	}*/
	
	// explore the arena and build the map
	private void buildMap(){
		getInformation();
		//Button.waitForAnyPress();
		int heading = getHeading();
		lastHeading = heading;
		frontDistance = getDistance();
		if ((heading % 180 != 0 && rightDistance > H_MOVE) || (heading % 180 == 0 && rightDistance > W_MOVE)) {
			if (!(heading==0&&robotW==0||heading==-90&&robotH==H_GRID-1||heading==180&&robotW==W_GRID-1||heading==90&&robotH==0)) {
				pilot.travel(5);
				pilot.rotate(90);
				if (getHeading() % 180 == 0) {
					move(H_MOVE);
				} else {
					move(W_MOVE);
					pilot.travel(-3);
				}
			}
		} else if ((heading%180==0 && frontDistance<H_MOVE) || (heading%180!=0 && frontDistance<W_MOVE)) {
			// also an obstacle on the left
			if (leftDistance < W_MOVE) {
				pilot.travel(frontDistance-RESERVED_DIST_H);
				pilot.rotate(-180);
				pilot.travel(-4);
			} else {
				pilot.travel(frontDistance-RESERVED_DIST_H);
				pilot.rotate(-90);
			}
		} else if (heading % 180 == 0) {
			move(H_MOVE);
		} else {
			move(W_MOVE);
		}
		// after returning to the start point, start to find the end point
		if (map.isMapFinished() && robotH==0 && robotW==0){
			readyToEnd = true;
		}
		drawMap();
	}
	
	// process the information (raw sensor data)
	private void processInformation(double distance, int relativeHeading) {
		int sensorHeading = (int) (relativeHeading - getHeading());
		if (sensorHeading <= -180) sensorHeading += 360;
		if (sensorHeading >= 180) sensorHeading = 360 - sensorHeading;
		int targetH=robotH, targetW=robotW;
		if (sensorHeading == 0) {
			targetH += 1 + (int)(distance/H_MOVE);
			//server.sendToClient("Detected Grid: ("+targetH+","+targetW+")");
			for (int h=robotH+1; h<targetH && h<=H_GRID-1 && h>=0; h++){
				server.sendToClient("Update false: " + h + "," + targetW + "\n");
				map.update(h, targetW, false);
			}
		} else if (sensorHeading == 180) {
			targetH -= 1 + (int)(distance/H_MOVE);
			//server.sendToClient("Detected Grid: ("+targetH+","+targetW+")");
			for (int h=targetH+1; h<robotH && h<=H_GRID-1 && h>=0; h++){
				//Button.waitForAnyPress();
				map.update(h, targetW, false);
			}
		} else if (sensorHeading == 90) {
			targetW += 1 + (int)(distance/W_MOVE);
			//server.sendToClient("Detected Grid: ("+targetH+","+targetW+")");
			for (int w=robotW+1; w<targetW && w<=W_GRID-1 && w>=0; w++){
				map.update(targetH, w, false);
			}
 		} else if (sensorHeading == -90) {
 			targetW -= 1 + (int)(distance/W_MOVE);
 			//server.sendToClient("Detected Grid: ("+targetH+","+targetW+")");
			for (int w=targetW+1; w<robotW && w<=W_GRID-1 && w>=0; w++){
				map.update(targetH, w, false);
			}
 		} 
		if (targetH<=H_GRID-1 && targetW<=W_GRID-1 && targetH>= 0 && targetW>= 0) {
			map.update(targetH,targetW, true);
		}
		drawMap();
	}
	
	// check whether the robot is really in a grid with blue paper
	// because sometimes the robot may go too far
	private boolean blueCheck() {
		int count = 0;
		float[] results = new float[5];
		pilot.travel(-1);
		results[0] = getColor();
		pilot.travel(-1);
		results[1] = getColor();
		pilot.travel(-1);
		results[2] = getColor();
		pilot.travel(1);
		results[3] = getColor();
		pilot.travel(1);
		results[4] = getColor();
		pilot.travel(1);
		for (int i=0; i<=4; i++) {
			if (results[i] <= BLUE_COLOR) count++;
		}
		if (count >= 3)
			return true;
		else 
			return false;
	}
	
	// check what color the grid is
	private void colorCheck() {
		// if blue is detected
		float colorDetected = getColor();
		if (robotH<=H_GRID-1&&robotH>=0&&robotW<=W_GRID-1&&robotW>=0){
			map.updatePassed(robotH, robotW);
		}
		if (colorDetected <= BLUE_COLOR){
			if (blueCheck() == true) {
				Sound.beep();
				map.updateEndPoint(new int[]{robotH,robotW});
				if (readyToEnd){
					taskFinished = true;
					Sound.twoBeeps();
					stop();
					return;
				}	
			}
		}
	}
	
	// get information from the environment using sensors
	private void getInformation() {
		drawMap();
		server.sendToClient("Step: " + (++step) + "\n");
		colorCheck();
		// first calibrate (adjust heading)
		uSensorMotor.rotate(-90);
		double lastRightDist = rightDistance;
		rightDistance = getDistance();
		server.sendToClient("Last=Right? " + (lastHeading==getHeading()));
		if (lastHeading == getHeading() && rightDistance < W_MOVE) {
			adjustAngleBetweenRightWall(lastRightDist, rightDistance);
			adjustRightAngle();
		}
		// second calibrate (adjust front distance)
		uSensorMotor.rotate(90);
		frontDistance = getDistance();
		if (frontDistance < HEIGHT_OF_ARENA){
			adjustDistanceBetweenFrontWall();
		}
		colorCheck();
		// now hopefully we can get the right information
		// front
		frontDistance = getDistance();
		processInformation(frontDistance, 0);
		// left
		uSensorMotor.rotate(90);
		leftDistance = getDistance();
		processInformation(leftDistance, 90);
		if (leftDistance < W_MOVE && (rightDistance>W_MOVE || readyToEnd)) {
			adjustLeftAngle();
			if (canMoveBack()) {
				leftDistance = getDistance();
				adjustLeftWallDistance();
			}
		}
		// front again
		uSensorMotor.rotate(-90);
		frontDistance = getDistance();
		processInformation(frontDistance, 0);
		// right
		uSensorMotor.rotate(-90);
		rightDistance = getDistance();
		processInformation(rightDistance, -90);
		server.sendToClient(makeMessage());
		if (rightDistance < W_MOVE) {
			adjustRightAngle();
			if (canMoveBack()) {
				rightDistance = getDistance();
				adjustRightWallDistance();
			}
			//adjustAngle();
		}
		uSensorMotor.rotate(90);
		drawMap();
	}

	// send essential data to the client (PC)
	private String makeMessage() {
		String message = "======================================================================\n";
		message += "Current Grid: (" + robotH + "," + robotW + ")\n"; 
		message += "Last Heading: " + lastHeading + "\n";
		message += "Heading: " + getHeading() + "\n";
		message += "front Distance: " + frontDistance + "\n";
		message += "Left Distance: " + leftDistance + "\n";
		message += "Right Distance: " + rightDistance + "\n";
		message += "Is at corner? " + robotIsAtCorner() + "\n";
		return message;
	}
	
	// adjust the angle between the robot and the left wall
	private void adjustLeftAngle(){
		double move = 5;
		double thres = 20;
		pilot.travel(-move);
		double lastLeft = getDistance();
		pilot.travel(move);
		double currentLeft = getDistance();
		double tantheta = (currentLeft-lastLeft)/THIRD_CALIBRATE_MOVE;
		double theta = Math.atan(tantheta)*180/Math.PI;
		if (Math.abs(theta)>thres) theta = 0;
		Pose oldPose = poseProvider.getPose();
		Pose copyPose = new Pose(oldPose.getX(), oldPose.getY(), oldPose.getHeading());
		pilot.rotate(-theta);
		poseProvider.setPose(copyPose);
	}
	
	// another version of adjustAngleBetweenRightWall method
	// to make sure the robot is parallel to the right wall
	private void adjustRightAngle(){
		double move = 5;
		double thres = 20;
		pilot.travel(-move);
		double lastRight = getDistance();
		pilot.travel(move);
		double currentRight = getDistance();
		double tantheta = (currentRight-lastRight)/THIRD_CALIBRATE_MOVE;
		double theta = Math.atan(tantheta)*180/Math.PI;
		if (Math.abs(theta)>thres) theta = 0;
		Pose oldPose = poseProvider.getPose();
		Pose copyPose = new Pose(oldPose.getX(), oldPose.getY(), oldPose.getHeading());
		pilot.rotate(theta);
		poseProvider.setPose(copyPose);
	}
	
	// the first calibration
	
	// the purpose of this calibration is to make the robot parallel to the right wall
	// this calibration is the basis for the other two calibrations
	private void adjustAngleBetweenRightWall(double lastRightDist, double rightDist){
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
		// if the theta is too large, there might be some problems...
		// we don't want the position provider know the robot has rotated becasue
		// this is just a calibration
		Pose oldPose = poseProvider.getPose();
		Pose copyPose = new Pose(oldPose.getX(), oldPose.getY(), oldPose.getHeading());
		if (Math.abs(theta) < 20) {
			pilot.rotate(theta);	
		}
		poseProvider.setPose(copyPose);
	}
	
	// the second calibration
	
	// the purpose of this calibration is to adjust the distance between the robot and the
	// front wall to make sure it is in the grid which it thinks it is in.
	private void adjustDistanceBetweenFrontWall() {
		double heading = getHeading();
		double remainder;
		double distanceToMove = 0;
		if (heading % 180 == 0) {
			remainder = frontDistance%H_MOVE;
			distanceToMove = remainder-RESERVED_DIST_H;
		} else {
			remainder = frontDistance % W_MOVE;
			distanceToMove = remainder-RESERVED_DIST_W;
		}
		if (distanceToMove < 8) {
			pilot.travel(distanceToMove);
		}
	}
	
	private boolean canMoveBack() {
		double heading = getHeading();
		int newH = robotH;
		int newW = robotW;
		if (heading == 0) {
			newH -= 1;
		} else if (heading == 180) {
			newH += 1;
		} else if (heading == 90) {
			newW += 1;
		} else if (heading == -90) {
			newW -= 1;
		}
		if (isWithinArena(newH, newW) && map.isOccupied(newH, newW)!=1) {
			return true;
		} else {
			return false;
		}
	}
	
	// the third calibration
	
	// a strange but effective method to adjust the distance between the robot and the right wall
	
	// when the distance between the robot and the right wall is too small, it would rotate left
	// for an angle, and move backwards (the grid behind must be empty) and then rotate right to
	// make it parallel to the right wall and then move forwards so that the robot keeps parallel
	// to the right wall and successfully makes the distance between it and the right wall shorter.
	
	// the robot would do a similar but reverse operation when the distance between it and the right
	// wall is too large
	
	// the purpose of this calibration to make sure the distance between the robot and the wall on its
	// right is within an acceptable range. We noticed that when the distance is too small, the robot
	// cannot rotate successfully while a large distance would make the distances detected by the ultrasonic
	// sensor not inaccurate.
	private void adjustRightWallDistance() {
		double dist;
		// calculate the difference between the expected distance and the real distance
		if (getHeading() % 180 == 0) {
			dist = rightDistance%W_MOVE-RESERVED_DIST_RIGHT;
		} else {
			dist = rightDistance%H_MOVE-RESERVED_DIST_RIGHT;
		}
		//server.sendToClient("Dist: " + dist + "\n");
		double sintheta = dist/THIRD_CALIBRATE_MOVE;
		//server.sendToClient("The sin theta: " + sintheta + "\n");
		// calculate the angle to rotate
		double theta;
		if (sintheta > 1) {
			theta = 85;
		} else {
			theta = Math.asin(sintheta)*180/Math.PI;
		}
		double threshold = 25;
		if (theta > threshold) theta = threshold;
		//server.sendToClient("The theta for third: " + theta + "\n");
		// when the calculated angle is too small, which means the distance is acceptable, skip this calibration
		if (Math.abs(theta) > 5){
			//server.sendToClient("The theta for third calibrate: " + theta);
			pilot.rotate(-theta);
			pilot.travel(-THIRD_CALIBRATE_MOVE);
			pilot.rotate(theta);
			pilot.travel(THIRD_CALIBRATE_MOVE);
		} 
	}
	
	private void adjustLeftWallDistance() {
		Sound.beep();
		double dist;
		// calculate the difference between the expected distance and the real distance
		if (getHeading() % 180 == 0) {
			dist = leftDistance%W_MOVE-RESERVED_DIST_RIGHT;
		} else {
			dist = leftDistance%H_MOVE-RESERVED_DIST_RIGHT;
		}
		//server.sendToClient("Dist: " + dist + "\n");
		double sintheta = dist/THIRD_CALIBRATE_MOVE;
		//server.sendToClient("The sin theta: " + sintheta + "\n");
		// calculate the angle to rotate
		double theta;
		if (sintheta > 1) {
			theta = 85;
		} else {
			theta = Math.asin(sintheta)*180/Math.PI;
		}
		if (theta > 15) theta = 15;
		//server.sendToClient("The theta for third: " + theta + "\n");
		// when the calculated angle is too small, which means the distance is acceptable, skip this calibration
		if (Math.abs(theta) > 5){
			//server.sendToClient("The theta for third calibrate: " + theta);
			pilot.rotate(theta);
			pilot.travel(-THIRD_CALIBRATE_MOVE);
			pilot.rotate(-theta);
			pilot.travel(THIRD_CALIBRATE_MOVE);
		} 
	}
	
	// stop the robot 
	public void stop() {
		arbitrator.stop();
		pilot.stop();
		taskFinished = true;
		Sound.beepSequenceUp();
		drawMap();
		while(true) {
			if (Button.getButtons() == Keys.ID_LEFT || Button.getButtons() == Keys.ID_UP) {
				drawMap();
			} else if (Button.getButtons() == Keys.ID_RIGHT || Button.getButtons() == Keys.ID_DOWN) {
				drawProbability();
			} else if (Button.getButtons() == Keys.ID_ESCAPE) {
				break;
			}
		}
		System.exit(0);
		//System.out.println("STOP");
	}

	public boolean isTaskFinished() {
		return taskFinished;
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
				} else if (map.isEndGridFound() && h==H_GRID-1-map.getEndPoint()[0] && w==W_GRID-1-map.getEndPoint()[1]){
					lcd.drawChar('O', bias+len*w+3, bias+len*h+2, GraphicsLCD.VCENTER);
				}
			}
		}
	}
	
	private void drawProbability() {
		lcd.clear();
		int bias = 5;
		int len = 19;
		int width = 30;
		for (int h=0; h<=H_GRID-1; h++) {
			for (int w=0; w<=W_GRID-1; w++) {
				lcd.drawRect(bias+width*w, bias+len*h, width, len);
				lcd.drawString(String.valueOf(df.format(map.getGrid(H_GRID-1-h, W_GRID-1-w))), bias+width*w+3, bias+len*h+2, GraphicsLCD.VCENTER);
			}
		}
	}
	

	// move to a grid following a path that is found by A* search
	private void navigateToGrid(int[] goal){
		while(robotH!=goal[0] || robotW!=goal[1]) {
			LinkedList<int[]> path = map.aStarPathFinding(new int[]{robotH,robotW}, goal);
			moveToGrid(path.get(0));
		}
		if (getColor() <= BLUE_COLOR) {
			taskFinished = true;
		} else {
			findEndPoint();
		}
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
		stoppingThread = new StoppingThread(this);
		stoppingThread.start();
		arbitrator.go();
	}
	
	public static void main(String[] args) {
		SmartRobot myRobot = new SmartRobot();
		myRobot.closeRobot();
	}
}
