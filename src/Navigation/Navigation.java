package Navigation;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;

import Application.RobotMovement;
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
	final static int FAST = 200, SLOW = 100, ACCELERATION = 4000;
	final static double DEG_ERR = 3.0, CM_ERR = 2, CM_ERR_2 = 1;
	

	//Board constants
	public static final int MAX_X_BOARD = -3;
	public static final int MIN_X_BOARD = 1;
	public static final int MAX_Y_BOARD = 3;
	public static final int MIN_Y_BOARD = 1;

	// ScanObject constants
	private static final int SCAN_TIME = 10;

	// Object avoidance constants
	private static final double MIN_DISTANCE = 6;
	private static final double SAFETY_RATIO = 1.7;
	private static final double SAFETY_ANGLE = 75;

	// isWooden constants
	private static final double DELTA_DISTANCE = 5;
	private static final double SCAN_ANGLE = 20;

	// temp variable, debug purposes
	// public static double[] data = new double[4];

	public static final int SLOW_ROTATE_SPEED = 50;
	public static final int FORWARD_SPEED = 150;
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
	 * heading
	 * This version of the method exits with boolean false if see an obstacle
	 */
	public boolean travelTo(double x, double y) {
		double minAng;
		
		
		
		minAng = getMinAng(x,y);
		this.turnTo(minAng, false);
		
		do {
			this.setSpeeds(FAST, FAST);
				if (us.getDistance() < MIN_DISTANCE ) {
					Sound.beepSequenceUp();
					return false;
			}		
		} while ((Math.abs(x - odometer.getX()) > CM_ERR_2 || 
				Math.abs(y - odometer.getY()) > CM_ERR_2) );
		
		
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
		double[] data = new double[4];
		boolean object = false;
		double[] blockProperties = new double[2];
	
		minAng = getMinAng(x,y);
		this.turnTo(minAng, false);
		
		while ((Math.abs(x - odometer.getX()) > CM_ERR || 
				Math.abs(y - odometer.getY()) > CM_ERR) 		&& (!object)) {


			this.setSpeeds(FAST, FAST);
			
			// if object too close, object avoidance
				if (us.getDistance() < MIN_DISTANCE && avoid) {
						//avoid
						avoidObject(true);

					travelTo(x, y, true);
				
			}
					//Sound.beep();
				
		}
		this.setSpeeds(0, 0);

	}
	public void avoidObject(boolean full) {
		//double[] data = new double[2];
		this.setSpeeds(0, 0);
		double angle = odometer.getTheta();
		//data = isWooden();

		Sound.beepSequenceUp();
		if(full){
			turnTo(angle + SAFETY_ANGLE, true);

		} else {
			turnTo(angle + 0.6 * SAFETY_ANGLE, true);
		}
		
		goForward(22 * SAFETY_RATIO);

		 turnTo(80, true);
		 
		 goForward(15);
			
		
		// turnTo(90, true);
	}

	public double getMinAng(double x, double y){
	double	minAng = (Math.atan2(y - odometer.getY(), x - odometer.getX())) * (180.0 / Math.PI);
		if (minAng < 0){
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

	//public double[] isWooden() {
	public double[] isWooden(){
		double minLow, minHigh, initialAngle;
		double[] results = new double[2];
		
		minLow = us.getDistance();
		minHigh = highUs.getDistance();
		initialAngle = odometer.getTheta();
		
		turnTo(initialAngle - SCAN_ANGLE, true);
		setSpeeds(-ROTATE_SPEED, ROTATE_SPEED);
		
		while(odometer.getTheta() < initialAngle + SCAN_ANGLE){
			if(us.getDistance() < minLow){
				minLow = us.getDistance();
			}
			
			if(highUs.getDistance() < minHigh){
				minHigh = highUs.getDistance();
			}
			
			try {
				Thread.sleep(SCAN_TIME/2);
			} catch (InterruptedException e) {
			//oh, boii, hope nothing happens here
				}
		}
		
		stop();
		
		if(minHigh < minLow + DELTA_DISTANCE){
			results[0] = 1; //wooden
		} else {
			results[0] = 0; //blue block
		}	
		
		if(results[0]==0){
			turnTo(initialAngle, true);
		}
		
		results[1] = minLow;
		
//		if(results[0] == 0 && minLow < 15){
	//		return false; // BLUE BLOCK 
		//} else { return true; } //WOODEN BLOCK OR DIDNT SEE BLUE BLOCK avoid jic
		
		
		return results;
	}

	
	public void rotateCounterClockwise(boolean fast) 
	{
		if(fast){
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
		} else {
			leftMotor.setSpeed((int)(0.5 * SLOW_ROTATE_SPEED));
			rightMotor.setSpeed((int)(0.5 * SLOW_ROTATE_SPEED));
		}
		odometer.setRotating(true);
		leftMotor.backward();
		rightMotor.forward();
	}

	public	void rotateClockwise(boolean fast) 
	{
		
		if(fast){
			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
		} else {
			leftMotor.setSpeed((int)0.5 * SLOW_ROTATE_SPEED);
			rightMotor.setSpeed((int)(0.5 * SLOW_ROTATE_SPEED));
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
}
