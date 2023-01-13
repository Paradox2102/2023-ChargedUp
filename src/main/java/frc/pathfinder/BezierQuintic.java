package frc.pathfinder;

import frc.pathfinder.Bezier.BezierPoint;

public class BezierQuintic 
{
	public BezierPoint m_p0, m_p1, m_p2, m_p3, m_p4, m_p5;
	
	public BezierQuintic(BezierPoint p0, BezierPoint p1, BezierPoint p2, BezierPoint p3, BezierPoint p4, BezierPoint p5)
	{
		m_p0 = p0;
		m_p1 = p1;
		m_p2 = p2;
		m_p3 = p3;
		m_p4 = p4;
		m_p5 = p5;
	}
	
	BezierQuintic(double p0x, double p0y, double angle1, double l1, double l3, double p3x, double p3y, double angle2, double l2, double l4)
	{
		if ((l1 == 0) || (l2 == 0))
		{
			double dx = p3x - p0x;
			double dy = p3y - p0y;
			double d = Math.sqrt(dx*dx + dy*dy);
			
			if (l1 == 0)
			{
				l1 = d / 2;
			}
			
			if (l2 == 0)
			{
				l2 = d / 3;
			}
		}
		
		double dx = l1 * Math.cos(angle1);
		double dy = l1 * Math.sin(angle1);

		m_p0 = new BezierPoint(p0x, p0y);
		m_p1 = new BezierPoint(p0x + dx, p0y + dy);
		
		if (l3 == 0)
		{
			m_p2 = m_p1;
		}
		else
		{
			dx = l3 * Math.cos(angle1);
			dy = l3 * Math.sin(angle1);
			
			m_p2 = new BezierPoint(p0x + dx, p0y + dy);
		}
		
		dx = l2 * Math.cos(angle2);
		dy = l2 * Math.sin(angle2);
		
		m_p4 = new BezierPoint(p3x + dx, p3y + dy);
		if (l4 == 0)
		{
			m_p3 = m_p4;
		}
		else
		{
			dx = l4 * Math.cos(angle2);
			dy = l4 * Math.sin(angle2);
			
			m_p3 = new BezierPoint(p3x + dx, p3y + dy);
		
		}
		
		m_p5 = new BezierPoint(p3x, p3y);
		
//		System.out.println(String.format("(%f,%f),(%f,%f),(%f,%f),(%f,%f),(%f,%f),(%f,%f)",
//				m_p0.m_x, m_p0.m_y, m_p1.m_x, m_p1.m_y, m_p2.m_x, m_p2.m_y, m_p3.m_x, m_p3.m_y, m_p4.m_x, m_p4.m_y, m_p5.m_x, m_p5.m_y));
	}
	
    public void ComputeSplinePoints(SplinePoint[] points, int first, int sample_count, double maxVelocity) 
    {
        double distance = 0;
        
    	for (int i = 0 ; i < sample_count ; i++)
    	{
    		double t = (double) i / sample_count;
    		double t2 = t * t;
    		double t3 = t2 * t;
    		double t4 = t3 * t;
    		double t5 = t4 * t;
    		double omt = (1 - t);
    		double omt2 = omt * omt;
    		double omt3 = omt2 * omt;
    		double omt4 = omt3 * omt;
    		double omt5 = omt4 * omt;
    		
    		double dx = 5*omt4*(m_p1.m_x-m_p0.m_x) + 20*omt3*t*(m_p2.m_x-m_p1.m_x) + 30*omt2*t2*(m_p3.m_x-m_p2.m_x) + 20*omt*t3*(m_p4.m_x-m_p3.m_x) + 5*t4*(m_p5.m_x-m_p4.m_x);
    		double dy = 5*omt4*(m_p1.m_y-m_p0.m_y) + 20*omt3*t*(m_p2.m_y-m_p1.m_y) + 30*omt2*t2*(m_p3.m_y-m_p2.m_y) + 20*omt*t3*(m_p4.m_y-m_p3.m_y) + 5*t4*(m_p5.m_y-m_p4.m_y);
    		
    		points[first + i] = new SplinePoint();
    		points[first + i].m_heading = Math.atan2(dy, dx);
    		points[first + i].m_x = omt5*m_p0.m_x + 5*omt4*t*m_p1.m_x + 10*omt3*t2*m_p2.m_x + 10*omt2*t3*m_p3.m_x + 5*omt*t4*m_p4.m_x + t5*m_p5.m_x;
    		points[first + i].m_y = omt5*m_p0.m_y + 5*omt4*t*m_p1.m_y + 10*omt3*t2*m_p2.m_y + 10*omt2*t3*m_p3.m_y + 5*omt*t4*m_p4.m_y + t5*m_p5.m_y;
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