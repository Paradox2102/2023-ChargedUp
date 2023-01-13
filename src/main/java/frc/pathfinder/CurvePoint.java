package frc.pathfinder;

public class CurvePoint
{
	public double m_x;
	public double m_y;
	public double m_heading;
	public double m_delta = 0;
	public double m_distance = 0;
	
	CurvePoint(double x, double y, double heading, double maxVelocity)
	{
		m_x = x;
		m_y = y;
		m_heading = heading;
	}
	
	CurvePoint()
	{
		m_x = 0;
		m_y = 0;
		m_heading = 0;
	}
}
