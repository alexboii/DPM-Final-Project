package Application;

public class Vector {
	
	private double distance;
	private double angle;
	private double initialX, initialY;
	
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

	//return XY values of final position of vector
	public double[] getFinalPosition(){
		double[] position = new double[2];
		
		position[0]= this.initialX + Math.cos(Math.toRadians(this.angle));
		position[0]= this.initialY + Math.sin(Math.toRadians(this.angle));

		return position;
	}
	
	
}
