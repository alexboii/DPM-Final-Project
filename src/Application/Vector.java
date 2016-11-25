package Application;

public class Vector {
	
	private double distance;
	private double angle;
	private double initialX, initialY;
	
	private static final int SAMPLES = 15;
	
	public Vector(double distance, double angle, double initialX, double initialY){
		this.distance = distance;
		this.angle = angle;
		this.initialX = initialX;
		this.initialY = initialY;
		
	}
	
	
	
	
	
	public double getInitialY() {
		return initialY;
	}
	
	public void setInitialY(double y){
		this.initialY = y;
	}
	
	public double getInitialX() {
		return initialX;
	}
	
	public void setInitialX(double x){
		this.initialX = x;
	}
	
	public double getDistance() {
		return distance;
	}

	public void setDistance(double distance) {
		this.distance = distance;
	}

	public double getAngle() {
		return angle;
	}

	public void setAngle(double angle) {
		this.angle = angle;
	}

	
	public boolean redZoneDetection(){
		for(int i = 0 ; i < SAMPLES; ++i){
				
			if(isInRedZone(this.getPointXY((this.getDistance() * i) / SAMPLES))){
				return false;
			}
		}
	return true;	
		
	}
	
	public boolean isInRedZone(double[] position){
		
		double LY, UY, LX, UX;
		
		LY =  StartRobot.LRZy;
		UY = StartRobot.URZy;
		LX = Math.abs(StartRobot.LRZx);
		UX = Math.abs(StartRobot.URZx);
		
		if( (LX < Math.abs(this.initialX) ) && ( Math.abs(this.initialX) < UX  ) 
		&& (LY < Math.abs(this.initialY) ) && ( Math.abs(this.initialY) < UY  ) ){
			return true; //in the red zone
		} else {
			return false; //not the red zone
		}
	}
	
	
	
	//return XY values of final position of vector
	public double[] getPointXY( double length){
		double[] position = new double[2];
		
		position[0]= Math.abs(this.initialX) + length *  Math.cos(Math.toRadians(this.angle));
		position[1]= this.initialY + length * Math.sin(Math.toRadians(this.angle));

		return position;
	}
	
	
}
