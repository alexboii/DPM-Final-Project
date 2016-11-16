package Application;

public class Vector {
	
	private double distance;
	private double angle;
	
	Vector(double distance, double angle){
		this.distance = distance;
		this.angle = angle;
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

	
}
