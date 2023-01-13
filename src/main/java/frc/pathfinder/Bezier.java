package frc.pathfinder;

public class Bezier 
{
	public static class BezierPoint
	{
		public double m_x;
		public double m_y;
		
		public BezierPoint(double x, double y)
		{
			m_x = x;
			m_y = y;
		}
	}
	
	public BezierPoint m_p0, m_p1, m_p2, m_p3;
	
	Bezier(BezierPoint p0, BezierPoint p1, BezierPoint p2, BezierPoint p3)
	{
		m_p0 = p0;
		m_p1 = p1;
		m_p2 = p2;
		m_p3 = p3;
	}
	
	Bezier(double p0x, double p0y, double angle1, double l1, double p3x, double p3y, double angle2, double l2)
	{
		double dx = l1 * Math.cos(angle1);
		double dy = l1 * Math.sin(angle1);

		m_p0 = new BezierPoint(p0x, p0y);
		m_p1 = new BezierPoint(p0x + dx, p0y + dy);
		
		dx = l2 * Math.cos(angle2);
		dy = l2 * Math.sin(angle2);
		
		m_p2 = new BezierPoint(p3x + dx, p3y + dy);
		m_p3 = new BezierPoint(p3x, p3y);
		
		// System.out.println(String.format("Bezier: (%f,%f),(%f,%f),(%f,%f),(%f,%f)",
		// 									m_p0.m_x, m_p0.m_y,
		// 									m_p1.m_x, m_p1.m_y,
		// 									m_p2.m_x, m_p2.m_y,
		// 									m_p3.m_x, m_p3.m_y));
		
	}
	
    public void ComputeSplinePoints(SplinePoint[] points, int first, int sample_count, double maxVelocity) 
    {
        double distance = 0;
        
    	for (int i = 0 ; i < sample_count ; i++)
    	{
    		double t = (double) i / sample_count;
    		double t2 = t * t;
    		double t3 = t * t * t;
    		double omt = (1 - t);
    		double omt2 = omt * omt;
    		double omt3 = omt2 * omt;
    		
    		double dx = 3*omt2*(m_p1.m_x - m_p0.m_x) + 6*omt*t*(m_p2.m_x - m_p1.m_x) + 3*t2*(m_p3.m_x - m_p2.m_x);
    		double dy = 3*omt2*(m_p1.m_y - m_p0.m_y) + 6*omt*t*(m_p2.m_y - m_p1.m_y) + 3*t2*(m_p3.m_y - m_p2.m_y);
    		
    		points[first + i] = new SplinePoint();
    		points[first + i].m_heading = Math.atan2(dy, dx);
    		points[first + i].m_x = omt3 * m_p0.m_x + 3*omt2*t * m_p1.m_x + 3*omt*t2 * m_p2.m_x + t3 * m_p3.m_x;
    		points[first + i].m_y = omt3 * m_p0.m_y + 3*omt2*t * m_p1.m_y + 3*omt*t2 * m_p2.m_y + t3 * m_p3.m_y;
    		points[first + i].m_maxVelocity = maxVelocity;
    		
            if ((i+first) > 0)
            {
	            dx = points[i+first].m_x - points[i+first-1].m_x;
	            dy = points[i+first].m_y - points[i+first-1].m_y;
	            double delta = Math.sqrt(dx*dx + dy*dy);
	            
	            points[i+first].m_delta = delta;
            
	            distance = points[i+first-1].m_distance + delta;
            }
            
            points[i+first].m_distance = distance;
    	}
    }
}