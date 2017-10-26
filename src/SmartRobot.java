import java.util.LinkedList;
import lejos.hardware.Brick;
import lejos.hardware.BrickFinder;
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
import lejos.hardware.Sound;
import lejos.hardware.lcd.GraphicsLCD;

// to do: add aidesoni to github repository

public class SmartRobot {
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
	private double rightDist, frontDist, leftDist;
	public int robot_h = 0;
	public int robot_w = 0;

	private static final double WIDTH = 150;
	private static final double HEIGHT = 193;
	private static final int H_GRID = 6;
	private static final int W_GRID = 6;
	private static final double W_MOVE = WIDTH / W_GRID;
	private static final double H_MOVE = HEIGHT / H_GRID;
	private OdometryPoseProvider poseProvider;
	
	private static final double rotateReserverdDistH = 11;
	private static final double rotateReservedDistW = 15;
	
	private boolean canCalibrate = false;

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
		Wheel leftWheel = WheeledChassis.modelWheel(Motor.B, 3.3).offset(-9.25);
		Wheel rightWheel = WheeledChassis.modelWheel(Motor.C, 3.3).offset(9.25);
		Chassis myChassis = new WheeledChassis(new Wheel[] { leftWheel, rightWheel }, WheeledChassis.TYPE_DIFFERENTIAL);
		pilot = new MovePilot(myChassis);
		pilot.setAngularSpeed(30);
		//pilot.setAngularAcceleration(5);
	}

	private void setupGridMap() {
		gridMap = new OccupanyGridMap(H_GRID, W_GRID);
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

	public void forward() {
		getInfo();
		int heading = getHeading();
		if ((heading % 180 != 0 && rightDist > H_MOVE) || (heading % 180 == 0 && rightDist > W_MOVE)) {
			pilot.rotate(90);
			if (getHeading() % 180 == 0) {
				move(H_MOVE);
			} else {
				move(W_MOVE);
			}
			canCalibrate = false;
		} else if ((heading % 180 == 0 && frontDist < H_MOVE) || (heading % 180 != 0 && frontDist < W_MOVE)) {
			System.out.println("Distance: " + (rotateReserverdDistH-frontDist));
			pilot.travel(frontDist-rotateReserverdDistH);
			pilot.rotate(-90);
			pilot.travel(-2);
			canCalibrate = false;
		} else if (heading % 180 == 0) {
			//System.out.println("MOVE HHHHHHHHHHH");
			if (robot_w == 0 || robot_w == W_GRID - 1){
				canCalibrate = true;
			}
			/*
			double tantheta = (rightDist - rotateReservedDistW)/H_MOVE;
			double theta = Math.atan(tantheta);
			double realMove = H_MOVE / Math.cos(theta);
			Pose oldPose = poseProvider.getPose();
			Button.waitForAnyPress();
			pilot.rotate(theta*180/Math.PI);
			poseProvider.setPose(oldPose);
			move(realMove);
			*/
			move(H_MOVE);
		} else {
			if (robot_h == 0 || robot_h == H_GRID - 1) {
				canCalibrate = true;
			}
			/*
			double tantheta = (rightDist - rotateReservedDistW)/W_MOVE;
			double theta = Math.atan(tantheta);
			double realMove = W_MOVE / Math.cos(theta);
			Pose oldPose = poseProvider.getPose();
			Button.waitForAnyPress();
			pilot.rotate(theta*180/Math.PI);
			poseProvider.setPose(oldPose);
			move(realMove);
			*/
			move(W_MOVE);
		}
	}
	
	public boolean robotIsAtCorner(){
		if ((robot_h==0&&robot_w==0)||(robot_h==0&&robot_w==W_GRID-1)||(robot_h==H_GRID-1&&robot_w==0)||(robot_h==H_GRID-1&&robot_w==W_GRID-1)) {
			return true;
		} else {
			return false;
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
			break;
		}
	}
	
	public void getInfo() {
		float color = getColor();
		boolean mapFinished = gridMap.isMapFinished();
		int[] robotGrid = new int[]{robot_h,robot_w};
		gridMap.updatePassed(robotGrid[0], robotGrid[1]);
		if (color < 0.05) {
			Sound.beep();
			if (mapFinished == true) {
				Sound.beep();
				stop();
			} else {
				gridMap.updateEndPoint(robotGrid);
			}
		}
		if (!mapFinished) {
			uSensorMotor.rotate(90);
			leftDist = getSingleDistance();
			//server.send("Left: " + leftDist);
			processInfo(leftDist, 90);
			uSensorMotor.rotate(-90);
			frontDist = getSingleDistance();
			//server.send("Front: " + frontDist);
			processInfo(frontDist, 0);
			uSensorMotor.rotate(-90);
			double lastRight = rightDist;
			rightDist = getSingleDistance();
			//server.send("Right: " + rightDist);
			processInfo(rightDist, -90);
			if (canCalibrate == true) {
				canCalibrate = false;
				double theta;
				double sintheta;
				double tantheta;
				double p;
				if (getHeading() % 180 == 0) {
					p = (rightDist - lastRight)/H_MOVE;			
				} else {
					p = (rightDist - lastRight)/W_MOVE;
				}
				sintheta = (-1 + Math.sqrt(1+4*Math.pow(p, 2)))/(2*p);
				tantheta = p;
				//theta = Math.atan(tantheta)*180/Math.PI;
			
				theta = Math.asin(sintheta)*180/Math.PI;
				Pose oldPose = poseProvider.getPose();
				pilot.rotate(theta);
				poseProvider.setPose(oldPose);
				
				if(!robotIsAtCorner()){
					oldPose = poseProvider.getPose();
					if (getHeading()%180==0){
						sintheta=(rotateReservedDistW-rightDist)/H_MOVE;
					} else{
						sintheta=(rotateReservedDistW-rightDist)/W_MOVE;
					}
					theta=Math.asin(sintheta);
					pilot.rotate(theta);
					poseProvider.setPose(oldPose);
				}
			}
			
			uSensorMotor.rotate(90);
			//rightDist = getSingleDistance();
			// second calibration
			
			frontDist = getSingleDistance();
			processInfo(frontDist, 0);
			}
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
		for (int h = H_GRID-1; h >=0; h--) {
			for (int w = W_GRID-1; w >= 0; w--) {
				lcd.drawRect(bias+len*w, bias+len*h, len, len);
			}
		}
		for (int hh = H_GRID-1; hh >= 0; hh--) {
			for (int ww = W_GRID-1; ww >= 0; ww--) {
				int h = H_GRID-1-hh;
				int w = W_GRID-1-ww;
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
	public SmartRobot() {
		ev3 = BrickFinder.getDefault();
		lcd = ev3.getGraphicsLCD();
		setupPilot();
		setupTouchSensor();
		setupColorSensor();
		setupUltrasonicSensor();
		setupGridMap();
		setupPoseProvider();
		setupBehaviours();
	}
	
	public void navigateToGrid(int[] goal){
		LinkedList<int[]> path = gridMap.aStarPathFinding(new int[]{robot_h,robot_w}, goal);
		for(int i=0; i<=path.size()-1;i++){
			moveToGrid(path.get(i));
		}
	}
	
	public void moveToGrid(int[] target){
		int heading = getHeading();
		getInfo();
		if (target[0] == robot_h+1) {
			// go up
			if (heading == 0){
				move(H_MOVE);
			} else if (heading == 180) {
				move(-H_MOVE);
			} else {
				pilot.rotate(-heading);
				move(H_MOVE);
			}
		} else if (target[0] == robot_h-1){
			// go down
			if (heading == 0){
				move(-H_MOVE);
			} else if (heading == 180) {
				move(H_MOVE);
			} else {
				pilot.rotate(heading);
				move(H_MOVE);
			}
		} else if (target[1] == robot_w+1) {
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
		} else if (target[1] == robot_w-1) {
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
		
	}
	
	public void followPath(LinkedList<int[]> thePath){
		for (int[] grid: thePath){
			moveToGrid(grid);
		}
	}
	
	public void findThePathToFindEndPoint(){
		while(true){
			LinkedList<int[]> bestExplorationPath = gridMap.findTheBestPathToExplore(new int[]{robot_h,robot_w});
			followPath(bestExplorationPath);
		}
	}
	
	public static void main(String[] args) {
		SmartRobot myRobot = new SmartRobot();
		new ThreadToClose(myRobot).start();
		myRobot.arbitrator.go();
		DrawingThread drawingThread = new DrawingThread(myRobot,20);
		drawingThread.start();
		myRobot.navigateToGrid(new int[]{0,5});
	}
}
