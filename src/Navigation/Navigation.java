package Navigation;

import Odometer.Odometer;
import SensorData.USPoller;
import lejos.hardware.motor.EV3LargeRegulatedMotor;

/**
 * This class allows the robot to travel to specific X and Y coordinates in the
 * arena, and also to turn to a desired angle from 0 to 360 degrees. It also
 * grants the robot the ability to approach an object by simply moving forward
 * towards it.
 * 
 * @author Sean Lawlor ECSE 211 - Design Principles and Methods, Head TA Fall
 *         2011 Ported to EV3 by: Francois Ouellet Delorme Fall 2015
 *
 */
public class Navigation {
	final static int FAST = 200, SLOW = -100, ACCELERATION = 4000;
	final static double DEG_ERR = 3.0, CM_ERR = 1.0;
	final static double EUCLIDEAN_ERROR = 5;
	private Odometer odometer;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	private USPoller usPoller;
	private EV3LargeRegulatedMotor usMotor;

	private static final int FORWARD_SPEED = 150;
	private static final int SLOW_ROTATE_SPEED = 40;


	public static final int ROTATE_SPEED = 50;
	private static final int BAND_WIDTH = 15;
	private static final double WALL_DISTANCE = 15;
	private static final int ANGLE_LIMIT = 110;
	private static final int BB_FAST_SPEED = 260;
	private static final int BB_SLOW_SPEED = 115;
	private static final int BB_OFFSET = 3;
	private static final int RIGHT_ANGLE = 90;

	/**
	 * Constructor
	 * 
	 * @param odo
	 *            Odometer
	 * @param usPoller
	 *            Ultrasonic Sensor Poller
	 */
	public Navigation(Odometer odo, USPoller usPoller) {
		this.odometer = odo;
		this.usPoller = usPoller;

		EV3LargeRegulatedMotor[] motors = this.odometer.getMotors();
		this.leftMotor = motors[0];
		this.rightMotor = motors[1];

		// set acceleration
		this.leftMotor.setAcceleration(ACCELERATION);
		this.rightMotor.setAcceleration(ACCELERATION);
	}

	/**
	 * Functions to set the motor speeds jointly
	 * 
	 * @param lSpd
	 *            Left Motor Speed
	 * @param rSpd
	 *            Right Motor Speed
	 */
	public void setSpeeds(float lSpd, float rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.forward();
		else
			this.leftMotor.backward();
		if (rSpd < 0)
			this.rightMotor.forward();
		else
			this.rightMotor.backward();
	}

	/**
	 * Functions to set the motor speeds jointly
	 * 
	 * @param lSpd
	 *            Left Motor Speed
	 * @param rSpd
	 *            Right Motor Speed
	 */
	public void setSpeeds(int lSpd, int rSpd) {
		this.leftMotor.setSpeed(lSpd);
		this.rightMotor.setSpeed(rSpd);
		if (lSpd < 0)
			this.leftMotor.forward();
		else
			this.leftMotor.backward();
		if (rSpd < 0)
			this.rightMotor.forward();
		else
			this.rightMotor.backward();
	}

	/**
	 * Float the two motors jointly
	 */
	public void setFloat() {
		this.leftMotor.stop();
		this.rightMotor.stop();
		this.leftMotor.flt(true);
		this.rightMotor.flt(true);
	}

	/**
	 * TravelTo function which takes as arguments the x and y position in cm
	 * Will travel to designated position, while constantly updating it's
	 * heading
	 * 
	 * @param x
	 * @param y
	 */
	 public void travelTo(double x, double y) {
		 double distance;
		 double dx, dy;
		 double error;

				

				dy = y - odometer.getY();
				dx = x - odometer.getX();


				double nextTheta = Math.atan2(dx, dy);

				if (nextTheta < 0) {
					nextTheta += Math.PI * 2;
				}
				
				error = Math.abs(nextTheta - odometer.getTheta());
				if (error > Math.toRadians(4.4)) {
					turnTo(nextTheta);
				}

				leftMotor.setSpeed(FORWARD_SPEED);
				rightMotor.setSpeed(FORWARD_SPEED);

				distance = Math.hypot(dx, dy);
			
				goForward(distance);
					
	//
//				currentX = odometer.getX();
//				currentY = odometer.getY();
//				currentTheta = odometer.getTheta();
				
			
			stop();
		}
				

		public void turnTo(double nextTheta) {
			double deltaTheta = odometer.getTheta() - nextTheta ;
			
			
			if (deltaTheta < -Math.PI) {
				deltaTheta += 2 * Math.PI;
		} else if (deltaTheta > Math.PI) {
				deltaTheta -= 2 * Math.PI;
			}

			leftMotor.setSpeed(ROTATE_SPEED);
			rightMotor.setSpeed(ROTATE_SPEED);
			leftMotor.rotate(-convertAngle(odometer.getRadius(), odometer.getTrack(), deltaTheta), true);
			rightMotor.rotate(convertAngle(odometer.getRadius(), odometer.getTrack(), deltaTheta), false);
			
		}
		

		public int convertDistance(double radius, double distance) {
			return (int) ((180.0 * distance) / (Math.PI * radius));
		}


		public int convertAngle(double radius, double width, double angle) {
			return convertDistance(radius, width *(angle) / 2);
		}
		
	/*
	 * Go foward a set distance in cm
	 */
	// public void goForward(double distance) {
	// this.travelTo(Math.cos(Math.toRadians(this.odometer.getTheta())) *
	// distance, Math.cos(Math.toRadians(this.odometer.getTheta())) * distance);
	//
	// }

	/**
	 * Go foward a set distance in cm
	 * 
	 * @param distance
	 *            Distance by which to go foward
	 */
	public void goForward(double distance) {
		//this.travelTo(odometer.getX() + Math.cos(Math.toRadians(this.odometer.getTheta())) * distance,
			//	odometer.getY() + Math.sin(Math.toRadians(this.odometer.getTheta())) * distance);

		
		if(distance != 0 ){
			leftMotor.rotate((int)(((distance*360) / (2 * odometer.getRadius() * Math.PI)) ), true);
			rightMotor.rotate((int)(((distance*360) / (2 * odometer.getRadius() * Math.PI)) ), false);
		} else {
			leftMotor.forward();
			rightMotor.forward();
		}
		
		
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
		leftMotor.forward();
		rightMotor.backward();
	}
	
	
	
	public void stop(){
		leftMotor.stop();
		rightMotor.stop();
	}

	/**
	 * @return Left Motor
	 */
	public EV3LargeRegulatedMotor getLeftMotor() {
		return leftMotor;
	}

	/**
	 * Set Left Motor
	 * 
	 * @param leftMotor
	 */
	public void setLeftMotor(EV3LargeRegulatedMotor leftMotor) {
		this.leftMotor = leftMotor;
	}

	/**
	 * @return Right Motor
	 */
	public EV3LargeRegulatedMotor getRightMotor() {
		return rightMotor;
	}

	/**
	 * Set Right Motor
	 * 
	 * @param rightMotor
	 */
	public void setRightMotor(EV3LargeRegulatedMotor rightMotor) {
		this.rightMotor = rightMotor;
	}
}
