/*
 * Odometer.java
 */

package Odometer;


import lejos.hardware.motor.EV3LargeRegulatedMotor;

public class Odometer extends Thread {
	// robot position
	private double x, y, theta;
	private int leftMotorTachoCount, rightMotorTachoCount;
	private EV3LargeRegulatedMotor leftMotor, rightMotor;
	// odometer update period, in ms
	private static final int ODOMETER_PERIOD = 25;
	//LCD update period
	
	private  double WHEEL_BASE = 16.8;
	private  double WHEEL_RADIUS = 2.1;
	
	/*variables*/
	private static int previousTachoL;          /* Tacho L at last sample */
	private static int previousTachoR;          /* Tacho R at last sample */
	private static int currentTachoL;           /* Current tacho L */
	private static int currentTachoR;           /* Current tacho R */
	


	

	// lock object for mutual exclusion
	private Object lock;

	// default constructor

	public Odometer(EV3LargeRegulatedMotor leftMotor,EV3LargeRegulatedMotor rightMotor) {
		this.leftMotor = leftMotor;
		this.rightMotor = rightMotor;
		this.x = 0.0;
		this.y = 0.0;
		this.theta = 0.0;
		leftMotor.resetTachoCount();
		rightMotor.resetTachoCount();
		this.leftMotorTachoCount = 0;
		this.rightMotorTachoCount = 0;
		lock = new Object();
		
		previousTachoL = 0;
		previousTachoR = 0;
		currentTachoL = 0;
		currentTachoR = 0;
		
	}
	

	// run method (required for Thread)
	public void run() {
		long updateStart, updateEnd;

		while (true) {
			updateStart = System.currentTimeMillis();
			// put (some of) your odometer code here
			double leftDistance, rightDistance, deltaDistance, deltaTheta, dX, dY;
			currentTachoL = leftMotor.getTachoCount();
			currentTachoR = rightMotor.getTachoCount();
			
			leftDistance = Math.PI * WHEEL_RADIUS * (currentTachoL - previousTachoL) / 180;
			rightDistance = Math.PI * WHEEL_RADIUS * (currentTachoR - previousTachoR) / 180;
			
			previousTachoL = currentTachoL;
			previousTachoR = currentTachoR;
			
			deltaDistance = .5 * (leftDistance + rightDistance);
			deltaTheta = (leftDistance - rightDistance) / WHEEL_BASE;

			synchronized (lock) {
				// don't use the variables x, y, or theta anywhere but here!
				theta += deltaTheta;
				
				dX = deltaDistance * Math.sin(theta);
				dY = deltaDistance * Math.cos(theta);
			
				
			//	theta = theta % (Math.PI * 2);

				
				theta= theta % (Math.PI * 2);
				if(theta < 0)
					theta = 2 * Math.PI + theta;
				
				
				x += dX;
				y += dY;
			}
						

			// this ensures that the odometer only runs once every period
			updateEnd = System.currentTimeMillis();
			if (updateEnd - updateStart < ODOMETER_PERIOD) {
				try {
					Thread.sleep(ODOMETER_PERIOD - (updateEnd - updateStart));
				} catch (InterruptedException e) {
					// there is nothing to be done here because it is not
					// expected that the odometer will be interrupted by
					// another thread
				}
			}
		}
	}
	
	
	
	public EV3LargeRegulatedMotor [] getMotors() {
		return new EV3LargeRegulatedMotor[] {this.leftMotor, this.rightMotor};
	}
	public EV3LargeRegulatedMotor getLeftMotor() {
		return this.leftMotor;
	}
	public EV3LargeRegulatedMotor getRightMotor() {
		return this.rightMotor;
	}	
		
		
	
	
	
	
	// return x,y,theta
	public void getPosition(double[] position) {
		synchronized (this) {
			position[0] = x;
			position[1] = y;
			position[2] = theta % (Math.PI * 2);
		}
	}


	// accessors
	public void getPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				position[0] = x;
			if (update[1])
				position[1] = y;
			if (update[2])
				position[2] = theta;
		}
	}

	public double getX() {
		double result;

		synchronized (lock) {
			result = x;
		}

		return result;
	}

	public double getY() {
		double result;

		synchronized (lock) {
			result = y;
		}

		return result;
	}

	public double getTheta() {
		double result;

		synchronized (lock) {
			result = theta;
		}

		return result;
	}

	// mutators
	public void setPosition(double[] position, boolean[] update) {
		// ensure that the values don't change while the odometer is running
		synchronized (lock) {
			if (update[0])
				x = position[0];
			if (update[1])
				y = position[1];
			if (update[2])
				theta = position[2];
		}
	}

	public void setX(double x) {
		synchronized (lock) {
			this.x = x;
		}
	}

	public void setY(double y) {
		synchronized (lock) {
			this.y = y;
		}
	}

	public void setTheta(double theta) {
		synchronized (lock) {
			this.theta = theta;
		}
	}
	
	public  double getTrack(){
		return this.WHEEL_BASE;
	}
	
	public  double getRadius(){
		return this.WHEEL_RADIUS;
	}
	

	/**
	 * @return the leftMotorTachoCount
	 */
	public int getLeftMotorTachoCount() {
		return leftMotorTachoCount;
	}

	/**
	 * @param leftMotorTachoCount the leftMotorTachoCount to set
	 */
	public void setLeftMotorTachoCount(int leftMotorTachoCount) {
		synchronized (lock) {
			this.leftMotorTachoCount = leftMotorTachoCount;	
		}
	}

	/**
	 * @return the rightMotorTachoCount
	 */
	public int getRightMotorTachoCount() {
		return rightMotorTachoCount;
	}

	/**
	 * @param rightMotorTachoCount the rightMotorTachoCount to set
	 */
	public void setRightMotorTachoCount(int rightMotorTachoCount) {
		synchronized (lock) {
			this.rightMotorTachoCount = rightMotorTachoCount;	
		}
	}
}
