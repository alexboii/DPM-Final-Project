package Navigation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import Application.RobotMovement;
import Application.StartRobot;
import Application.Vector;
/*
 * File: Navigation.java
 * Written by: Sean Lawlor
 * ECSE 211 - Design Principles and Methods, Head TA
 * Fall 2011
 * Ported to EV3 by: Francois Ouellet Delorme
 * Fall 2015
 * 
 * Movement control class (turnTo, travelTo, flt, localize)
 */
import Odometer.Odometer;
import SensorData.USPoller;
import lejos.hardware.Sound;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Navigation {
	final static int FAST = 200, SLOW = 180, ACCELERATION = 4000;
	final static double DEG_ERR = 3.0, CM_ERR = 2, CM_ERR_2 = 1;

	// Board constants
	public static final int MAX_X_BOARD = -3;
	public static final int MIN_X_BOARD = 1;
	public static final int MAX_Y_BOARD = 3;
	public static final int MIN_Y_BOARD = 1;

	// ScanObject constants
	private static final int SCAN_TIME = 10;

	// Object avoidance constants
	private static final double MIN_DISTANCE = 6;
	private static final double SAFETY_RATIO = 1;
	private static final double SAFETY_ANGLE = 50;

	// isWooden constants
	private static final double DELTA_DISTANCE = 9;
	private static final double SCAN_ANGLE = 20;
	private static final int DETECTION_ANGLE = 0;

	// temp variable, debug purposes
	// public static double[] data = new double[4];

	public static final int SLOW_ROTATE_SPEED = 50;
	public static final int FORWARD_SPEED = 200;
	public static final int WAYPOINT_BLOCKED_BW = 3;

	public static final int ROTATE_SPEED = 80;

	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private USPoller us, highUs;

	public Navigation(Odometer odo, USPoller us, USPoller highUs) {
		this.odometer = odo;
		this.us = us;
		this.highUs = highUs;
		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];

		// set acceleration
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
	}

	/*
	 * Functions to set the motor speeds jointly
	 */
	public void setSpeeds(float lSpd, float rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}

	public void setSpeeds(int lSpd, int rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.backward();
		else
			this.leftMotor.forward();
		if (rSpd < 0)
			this.rightMotor.backward();
		else
			this.rightMotor.forward();
	}

	public void waitMs(long time) {
		try {
			Thread.sleep(time);
		} catch (InterruptedException e) {
			// Do nothing, it should never happen
			e.printStackTrace();
		}
	}

	/*
	 * Float the two motors jointly
	 */
	public void setFloat() {
		this.leftMotor.stop();
		this.rightMotor.stop();
		this.leftMotor.flt(true);
		this.rightMotor.flt(true);
	}

	/*
	 * TravelTo function which takes as arguments the x and y position in cm
	 * Will travel to designated position, while constantly updating it's
	 * heading This version of the method exits with boolean false if see an
	 * obstacle
	 */
	public boolean travelTo(double x, double y) {
		double minAng;

		minAng = getMinAng(x, y);
		this.turnTo(minAng, true);

		do {
			this.setSpeeds(FAST, FAST);
			if (us.getDistance() < MIN_DISTANCE) {
				Sound.beepSequenceUp();
				return false;
			}
		} while ( Math.abs(Math.abs(x) - Math.abs(odometer.getX())) > CM_ERR 
				
				|| Math.abs(y - odometer.getY()) > CM_ERR);

		this.setSpeeds(0, 0);
		return true;
	}

	/*
	 * TravelTo function which takes as arguments the x and y position in cm
	 * Will travel to designated position, while constantly updating it's
	 * heading
	 */
	public void travelTo(double x, double y, boolean avoid) {
		double minAng;
		//double[] data = new double[4];
		boolean object = false;
	//	double[] blockProperties = new double[2];
		boolean nearForbiddenZone = false;

		minAng = getMinAng(x, y);
		this.turnTo(minAng, true);

		while ((Math.abs(x - odometer.getX()) > CM_ERR || Math.abs(y - odometer.getY()) > CM_ERR) ) {

			this.setSpeeds(FAST, FAST);

			// if object too close, object avoidance
			if (us.getDistance() < MIN_DISTANCE && avoid) {
				// avoid
				avoidObject(true);
				travelTo(x, y, true);

			}

			if (((odometer.getX() + 10) > StartRobot.LFZx || (odometer.getX() + 10) > StartRobot.UFZx)
					&& ((odometer.getY() + 10) > StartRobot.LFZy || (odometer.getY() + 10) > StartRobot.UFZy)) {
				nearForbiddenZone = true;
				break;
			}

		}

		this.setSpeeds(0, 0);

		if (nearForbiddenZone) {
		    double[][] points = new double[4][2];
		    points[0][0] = StartRobot.LFZx;
		    points[0][1] = StartRobot.LFZy;
		    points[1][0] = StartRobot.UFZx;
		    points[1][1] = StartRobot.LFZy;
		    points[2][0] = StartRobot.LFZx;
		    points[2][1] = StartRobot.UFZy;
		    points[3][0] = StartRobot.UFZx;
		    points[3][1] = StartRobot.UFZy;


		    double distanceLowerCorner1 = distance(x, y, points[0][0], points[0][1]); // Initialize shortestDistance
		    double distanceLowerCorner2 = distance(x, y, points[1][0], points[1][1]); 
		    double distanceUpperCorner1 = distance(x, y, points[2][0], points[2][1]); 
		    double distanceUpperCorner2 = distance(x, y, points[3][0], points[3][1]); 
		    ArrayList<Double> distances = new ArrayList<Double>();
		    distances.add(distanceLowerCorner1);
		    distances.add(distanceLowerCorner2);
		    distances.add(distanceUpperCorner1);
		    distances.add(distanceUpperCorner2);
		    
		    Collections.sort(distances);

		    if(distances.get(0) == distanceLowerCorner1){
		    	
		    	if(Math.abs(odometer.getX() - points[0][0]) < Math.abs(odometer.getY() - points[0][1])){
		    		travelTo(points[0][0], odometer.getY(), true);
		    		travelTo(odometer.getX(), points[0][1], true);
		    		travelTo(x, y);
		    	}else{
		    		travelTo(odometer.getX(), points[0][1], true);
		    		travelTo(points[0][0], odometer.getY(), true);
		    		travelTo(x, y);
		    	}
		    	
		    }

		    if(distances.get(0) == distanceLowerCorner2){
		    	if(Math.abs(odometer.getX() - points[1][0]) < Math.abs(odometer.getY() - points[1][1])){
		    		travelTo(points[1][0], odometer.getY(), true);
		    		travelTo(odometer.getX(), points[1][1], true);
		    		travelTo(x, y);
		    	}else{
		    		travelTo(odometer.getX(), points[1][1], true);
		    		travelTo(points[1][0], odometer.getY(), true);
		    		travelTo(x, y);
		    	}
		    }

		    if(distances.get(0) == distanceUpperCorner1){
		    	if(Math.abs(odometer.getX() - points[2][0]) < Math.abs(odometer.getY() - points[2][1])){
		    		travelTo(points[2][0], odometer.getY(), true);
		    		travelTo(odometer.getX(), points[2][1], true);
		    		travelTo(x, y);
		    	}else{
		    		travelTo(odometer.getX(), points[2][1], true);
		    		travelTo(points[2][0], odometer.getY(), true);
		    		travelTo(x, y);
		    	}
		    }

		    if(distances.get(0) == distanceUpperCorner2){
		    	if(Math.abs(odometer.getX() - points[3][0]) < Math.abs(odometer.getY() - points[3][1])){
		    		travelTo(points[3][0], odometer.getY(), true);
		    		travelTo(odometer.getX(), points[3][1], true);
		    		travelTo(x, y);
		    	}else{
		    		travelTo(odometer.getX(), points[3][1], true);
		    		travelTo(points[3][0], odometer.getY(), true);
		    		travelTo(x, y);
		    	}
		    }


//
//		    
//			double checkYCoordinateWithinRedZone = (y) - (y - odometer.getY() + 13);
//			if (checkYCoordinateWithinRedZone > StartRobot.LFZy && checkYCoordinateWithinRedZone < StartRobot.UFZy) {
//				travelTo(x, odometer.getY());
//				travelTo(odometer.getX(), y);
//				travelTo(x, y);
//			}else{
//				travelTo(y, odometer.getX());
//				travelTo(odometer.getY(), y);
//				travelTo(x, y);
//			}

		}
	}
	
	

	public void avoidObject(boolean full) {
		// double[] data = new double[2];
		this.setSpeeds(0, 0);
		double initialAngle = odometer.getTheta();
		double angle;
		this.setSpeeds(-150, 150);

		while (true) {
			if (us.getDistance() > 30) {
				angle = odometer.getTheta();
				break;
			}
		}
		this.setSpeeds(0, 0);

		Sound.beepSequenceUp();
		if (full) {
			turnTo(angle + SAFETY_ANGLE, true);		

		} else {
			turnTo(angle + 0.6 * SAFETY_ANGLE, true);
		}

		goForward(30);
		//Vector vector = new Vector(30 * SAFETY_RATIO, odometer.getTheta(), Math.abs(odometer.getX()), odometer.getY());

		//double pointXY1[] = vector.getPointXY(vector.getDistance());
		//travelTo(-Math.abs(pointXY1[0]), pointXY1[1], true);
		
		turnTo(initialAngle, true);
		goForward(5);
		
		
	//	turnTo(130, true);

	//	vector = new Vector(20, odometer.getTheta(), Math.abs(odometer.getX()), odometer.getY());

	//	double pointXY2[] = vector.getPointXY(vector.getDistance());
	//	travelTo(pointXY2[0], pointXY2[1], true);

	}

	public double getMinAng(double x, double y) {
		double minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0 / Math.PI);
		if (minAng < 0) {
			minAng += 360.0;
		}
		return minAng;
	}

	/*
	 * TurnTo function which takes an angle and boolean as arguments The boolean
	 * controls whether or not to stop the motors when the turn is completed
	 */
	public void turnTo(double angle, boolean stop) {

		double error = angle - this.odometer.getTheta();

		while (Math.abs(error) > DEG_ERR) {
			odometer.setRotating(true);
			error = angle - this.odometer.getTheta();

			if (error < -180.0) {
				this.setSpeeds(-SLOW, SLOW);
			} else if (error < 0.0) {
				this.setSpeeds(SLOW, -SLOW);
			} else if (error > 180.0) {
				this.setSpeeds(SLOW, -SLOW);
			} else {
				this.setSpeeds(-SLOW, SLOW);
			}
		}
		odometer.setRotating(false);
		if (stop) {
			this.setSpeeds(0, 0);
		}
	}

	// public double[] isWooden() {
	public double[] isWooden() {
		double minLow, minHigh, initialAngle;
	//	Vector vector;
		double[] results = new double[2];

		minLow = us.getDistance();
		minHigh = highUs.getDistance();
		initialAngle = odometer.getTheta();
		double bestAngle = initialAngle;

		turnTo(initialAngle - SCAN_ANGLE, true);
		setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);

		while (odometer.getTheta() < initialAngle + SCAN_ANGLE) {
			if (us.getFilteredDistance() < minLow) {
				minLow = us.getFilteredDistance();
				bestAngle = odometer.getTheta();
			}

			if (highUs.getFilteredDistance() < minHigh) {
				minHigh = highUs.getFilteredDistance();
			}

			try {
				Thread.sleep(SCAN_TIME / 2);	
			} catch (InterruptedException e) {
				// oh, boii, hope nothing happens here
			}
		}

		stop();

		if (minHigh < minLow + DELTA_DISTANCE) {
			results[0] = 1; // wooden
		} else {
			results[0] = 0; // blue block
		}

		if (results[0] == 0) {
			
			if(bestAngle < initialAngle){
				turnTo(bestAngle - DETECTION_ANGLE , true);
			} else {
				turnTo(bestAngle + DETECTION_ANGLE , true);
			}		
		}

		results[1] = minLow;

		// if(results[0] == 0 && minLow < 15){
		// return false; // BLUE BLOCK
		// } else { return true; } //WOODEN BLOCK OR DIDNT SEE BLUE BLOCK avoid
		// jic

		return results;
	}

	public void rotateCounterClockwise(boolean fast) {
		if (fast) {
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
		} else {
			leftMotor.setSpeed((int) (0.5 * SLOW_ROTATE_SPEED));
			rightMotor.setSpeed((int) (0.5 * SLOW_ROTATE_SPEED));
		}
		odometer.setRotating(true);
		leftMotor.backward();
		rightMotor.forward();
	}

	public void rotateClockwise(boolean fast) {

		if (fast) {
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
		} else {
			leftMotor.setSpeed((int) 0.5 * SLOW_ROTATE_SPEED);
			rightMotor.setSpeed((int) (0.5 * SLOW_ROTATE_SPEED));
		}
		odometer.setRotating(true);
		leftMotor.forward();
		rightMotor.backward();
	}

	public void goForward(double distance) {
		// this.travelTo(odometer.getX() +
		// Math.cos(Math.toRadians(this.odometer.getTheta())) * distance,
		// odometer.getY() + Math.sin(Math.toRadians(this.odometer.getTheta()))
		// * distance, false);

		this.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);

		leftMotor.rotate((int) ((180.0 * distance) / (Math.PI * 2.1)), true);
		rightMotor.rotate((int) ((180.0 * distance) / (Math.PI * 2.1)), false);
	}

	public void stop() {

		leftMotor.stop();
		rightMotor.stop();
		odometer.setRotating(false);
	}

	public EV3LargeRegulatedMotor getLeftMotor() {
		return leftMotor;
	}

	public void setLeftMotor(EV3LargeRegulatedMotor leftMotor) {
		this.leftMotor = leftMotor;
	}

	public EV3LargeRegulatedMotor getRightMotor() {
		return rightMotor;
	}

	public void setRightMotor(EV3LargeRegulatedMotor rightMotor) {
		this.rightMotor = rightMotor;
	}
	
	  /** Compute the distance between two points (x1, y1) and (x2, y2)*/
	  public static double distance(
	      double x1, double y1, double x2, double y2) {
	    return Math.sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
	  }

	
	
}
