package Odometer;

/**
 * This class will correct any inaccuracy in the odometer caused by factors such
 * as wheel slippage and others, with the help of a light sensor. This method
 * attempts to correct the odometer values based on the occurrence of lines on
 * the board.
 * 
 * @author Sebastian
 *
 */
public class OdometerCorrection {

	private double x, y;
	private Odometer odo;
	private final double MARGIN = 5;
	private final double TILE = 30.48;

	/**
	 * Constructor
	 * 
	 * @param odo
	 *            Odometer
	 */
	public OdometerCorrection(Odometer odo) {
		this.odo = odo;
	}

	/**
	 * Set Odometer
	 * 
	 * @param odo
	 *            Odometer
	 */
	public void setOdometer(Odometer odo) {
		this.odo = odo;
	}

	/**
	 * Perform odometry correction
	 */
	public void correct() {

		double[] position = new double[3];
		boolean[] update = new boolean[3];

		x = odo.getX();
		y = odo.getY();

		double deltaX = (x) % TILE;
		double deltaY = (y) % TILE;

		update[0] = (deltaX < MARGIN);
		update[1] = (deltaY < MARGIN);

		position[0] = ((int) (x / TILE)) * TILE;
		position[1] = ((int) (y / TILE)) * TILE;

		odo.setPosition(position, update);

	}
}
