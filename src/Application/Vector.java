
package Application;

import Odometer.LCDInfo;

/**
 * 
 * Class that represents Vectors in the XY plane. Counts with methods to
 * calculate the XY values of any point on the line of the vector.
 * 
 * 
 * @author Sebastian Andrade and Alexander Bratyshkin
 * 
 */
public class Vector {

	private double distance;
	private double angle;
	private double initialX, initialY;

	private static final int SAMPLES = 15;

	/**
	 * Creates a vector object
	 * 
	 * @param distance
	 *            length of vector
	 * @param angle
	 *            of the vector
	 * @param initialX
	 *            initial x value
	 * @param initialY
	 *            initial y value
	 */
	public Vector(double distance, double angle, double initialX, double initialY) {
		this.distance = distance;
		this.angle = angle;
		this.initialX = initialX;
		this.initialY = initialY;

	}

	/**
	 * @return initial y position of the tail of the vector
	 */
	public double getInitialY() {
		return initialY;
	}

	/**
	 * @param y
	 *            initial y position of the tail of the vector
	 */
	public void setInitialY(double y) {
		this.initialY = y;
	}

	/**
	 * @return initial x position of the tail of the vector
	 */
	public double getInitialX() {
		return initialX;
	}

	/**
	 * @param x
	 *            initial x position of the tail of the vector
	 */
	public void setInitialX(double x) {
		this.initialX = x;
	}

	/**
	 * @return
	 */
	public double getDistance() {
		return distance;
	}

	/**
	 * @param distance
	 *            of the vector
	 */
	public void setDistance(double distance) {
		this.distance = distance;
	}

	/**
	 * @return angle of the vector
	 */
	public double getAngle() {
		return angle;
	}

	/**
	 * 
	 * @param angle
	 *            of the vector
	 */
	public void setAngle(double angle) {
		this.angle = angle;
	}

	/**
	 * @return true if any of the selected points of the vector go fall in the
	 *         red zone
	 */
	public boolean redZoneDetection() {
		for (int i = 0; i < SAMPLES; ++i) {

			if (isInRedZone(this.getPointXY((this.getDistance() * i) / SAMPLES))) {
				return false;
			}
		}
		return true;

	}

	/**
	 * @param position
	 *            array of doubles containing the current XY values of the
	 *            odometer
	 * @return true if inside of red zone
	 */
	public boolean isInRedZone(double[] position) {

		double LY, UY, LX, UX;

		LY = StartRobot.LRZy;
		UY = StartRobot.URZy;
		LX = Math.abs(StartRobot.LRZx);
		UX = Math.abs(StartRobot.URZx);

		if ((LX < Math.abs(this.initialX)) && (Math.abs(this.initialX) < UX) && (LY < Math.abs(this.initialY))
				&& (Math.abs(this.initialY) < UY)) {
			return true;
		} else {
			return false;
		}
	}

	/**
	 * @param length
	 *            of the desired vector
	 * @return XY values of final position of vector
	 */
	public double[] getPointXY(double length) {
		double[] position = new double[2];

		position[0] = Math.abs(this.initialX) + length * Math.cos(Math.toRadians(this.angle));
		position[1] = this.initialY + length * Math.sin(Math.toRadians(this.angle));

		return position;
	}

}
