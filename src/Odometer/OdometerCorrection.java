package Odometer;

import lejos.hardware.Sound;

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

	// Spacing of the tiles in centimeters
	private static final double TILE_SPACING = 30.48;
	// Half the said spacing
	private static final double HALF_TILE_SPACING = TILE_SPACING / 2;
	// various pi ratios
	private static final double TWO_PI = Math.PI * 2;
	private static final double ONE_QUARTER_PI = Math.PI / 4;
	private static final double THREE_QUARTER_PI = 3 * ONE_QUARTER_PI;
	private static final double FIVE_QUARTER_PI = 5 * ONE_QUARTER_PI;
	private static final double SEVEN_QUARTER_PI = 7 * ONE_QUARTER_PI;
	
	private static final double SENSOR_OFFSET = 11;

	
	private static double Xc;
	private static double Yc;
	
	
	
	private double x, y;
	private Odometer odometer;
	private final double MARGIN = 5;
	private final double TILE = 30.48;

	/**
	 * Constructor
	 * 
	 * @param odo
	 *            Odometer
	 */
	public OdometerCorrection(Odometer odo) {
		this.odometer = odo;
	}

	/**
	 * Set Odometer
	 * 
	 * @param odo
	 *            Odometer
	 */
	public void setOdometer(Odometer odo) {
		this.odometer = odo;
	}

	
//	// CHECK THETA ANGLE OF THE ROBOT TO DETERMINE POSITION OF LINES
//	public boolean isMovingHorizontally(double theta) {
//		return theta >= ONE_QUARTER_PI && theta < THREE_QUARTER_PI || theta >= FIVE_QUARTER_PI && theta < SEVEN_QUARTER_PI;
//	}
	
	
	public static void setXc(double x){
		Xc = x;
	}
	
	
	public static void setYc(double d){
		Yc = d;
	}

	
	
	/**
	 * Perform odometry correction
	 */
	
	
	
	public void CorrectorFormula(){
		
		
		
		if(!this.odometer.isRotating() && 
				(this.odometer.getLeftMotor().isMoving() || this.odometer.getRightMotor().isMoving() )){
			
			// Sound.beep();
			// wrap theta to 0 <= theta < 2i
			double theta = Math.toRadians(odometer.getTheta());
			// check which line direction we just crossed using the heading
			double sensorYOffset = Math.sin(theta -Math.PI) * SENSOR_OFFSET;
			double sensorXOffset = Math.cos(theta- Math.PI) * SENSOR_OFFSET;

			
			if( Math.abs(sensorYOffset) > Math.abs(sensorXOffset)   ){
			//if (theta >= ONE_QUARTER_PI && theta < THREE_QUARTER_PI || theta >= FIVE_QUARTER_PI && theta < SEVEN_QUARTER_PI) {
			//	Sound.playNote(Sound.FLUTE, 440, 250);
				// cross horizontal line
				// offset y to account for sensor distance
				double y =  odometer.getY() + sensorYOffset;
				// snap y to closest line
				y = Math.round((y + sensorYOffset) / TILE_SPACING) * TILE_SPACING - HALF_TILE_SPACING;
				// correct y, removing the offset
			//	Sound.playNote(Sound.FLUTE, 440, 250);
				
				setYc(y - sensorYOffset );
				odometer.setY(y - sensorYOffset/2);
				
			} else {
			//	Sound.playNote(Sound.FLUTE, 880, 250);
				// cross vertical line
				// offset x to account for sensor distance
				double x =  odometer.getX() - sensorXOffset;
				// snap x to closest line
				x = Math.round((x + sensorXOffset) / TILE_SPACING) * TILE_SPACING + HALF_TILE_SPACING;
				// correct x, removing the offset
			//	Sound.playNote(Sound.FLUTE, 440, 250);
	
				
				setXc(-(x + sensorXOffset ));
				odometer.setX((x - sensorXOffset/2 ));
			}
			// set the line as crossed to prevent repeated events
		
		}
		
		
	}
	
	
	
}
