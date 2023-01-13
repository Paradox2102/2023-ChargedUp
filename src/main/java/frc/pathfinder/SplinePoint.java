package frc.pathfinder;

public class SplinePoint
{
	public double m_x;
	public double m_y;
	public double m_heading;
	public double m_delta = 0;
	public double m_distance = 0;
	public double m_maxVelocity;
	
	SplinePoint(double x, double y, double heading, double maxVelocity)
	{
		m_x = x;
		m_y = y;
		m_heading = heading;
		m_maxVelocity = maxVelocity;
	}
	
	SplinePoint()
	{
		m_x = 0;
		m_y = 0;
		m_heading = 0;
		m_maxVelocity = 0;
	}
}