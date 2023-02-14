package frc.pathfinder;

public class MathUtil 
{
	public static double PI = 3.14159265358979323846;
	public static double TAU = PI*2;
	
	public static double bound_radians(double angle) {
	    double newAngle = angle % TAU;	//fmod(angle, TAU);
	    if (newAngle < 0) newAngle = TAU + newAngle;
	    return newAngle;
	}
	
	public static double r2d(double angleInRads) {
	    return angleInRads * 180 / PI;
	}
	
	public static double d2r(double angleInDegrees) {
	    return angleInDegrees * PI / 180;
	}
	
	public static double normalizeRadians(double angle)
	{
		if (angle > Math.PI)
		{
			angle -= 2 * Math.PI;
		}
		else if (angle < -Math.PI)
		{
			angle += 2 * Math.PI;
		}
		
		return(angle);
	}
	
	public static double normalizeDegrees(double angle)
	{
		angle = angle%360;
		if (angle > 180)
		{
			angle -= 360;
		}
		else if (angle < -180)
		{
			angle += 360;
		}
		
		return(angle);
	}
}