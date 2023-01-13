/*
 *	  Copyright (C) 2021  John H. Gaby
 *
 *    This program is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, version 3 of the License.
 *
 *    This program is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *    
 *    Contact: robotics@gabysoft.com
 */
package frc.pathfinder;

/**
 * 
 * @author John Gaby
 * 
 * @brief The Pathfinder class is used to convert waypoints into a Path that
 * 			can be used by the PurePursuit class to control the robot so that
 * 			it follows a specified path.
 * 
 */
public class Pathfinder 
{
	/*
	 * These are the default values used by the PurePursuit code
	 */
	private static double k_lookAheadTime = 0.75;		// Time along the path of the next target point	
	private static double k_minLookAheadDist = 0.6;		// Minimum allowed look ahead distance (overrides look ahead time if necessary)
	private static double k_maxSearchTime = 1;			// Max time along the path to search for the closest point to the current position
	private static double k_minSpeed = 0.50;			// Minimum allowed speed
	private static double k_extendedLookAheadDistance = .75;	// Distance to extend path if m_isExtended is true
	private static double k_curvatureAdjust = 1;		// Adjusts the curvature to make the path following more or less aggressive
														//   A value of 1 is normal, a value greater than 1 makes the 
														//   path following more aggressive, less than 1 makes it less aggressive

	/**
	 * Sets the default lookahead time that is used when new paths are created.
	 * 	This is the time that PurePursuit looks ahead on the path to find the point at which to steer.
	 * 
	 * @param lookAheadTime - Specifies lookahead time in seconds.
	 */
	public static void  setDefaultLookAheadTime(double lookAheadTime)
	{
		k_lookAheadTime = lookAheadTime;
	}

	/**
	 * Sets the default minimum lookahead distance that is used when new paths are created
	 * 	If the distance computed by using the lookAheadTime is less than this value,
	 *  then this value will be used instead.
	 * 
	 * @param minLookAheadDist - Specifies the minimum lookahead distance in feet.
	 */
	public static void setDefaultMinLookAheadDistance(double minLookAheadDist) {
		k_minLookAheadDist = minLookAheadDist;
	}

	/**
	 * Sets the default max search time when new paths are created.
	 *  When searching for the closest point on the path to the current robot
	 *  position, this is that max time from the current time to search.
	 * 
	 * @param maxSearchTime - Specifies the max search time in seconds.
	 */
	public static void setDefaultMaxSearchTime(double maxSearchTime) {
		k_maxSearchTime = maxSearchTime;
	}

	/**
	 * Sets the default minimum speed for the robot when new paths are created.
	 * 
	 * @param minSpeed - Specifies the minimum speed for the robot in feet/second.
	 */
	public static void setDefaultMinSpeed(double minSpeed) {
		k_minSpeed = minSpeed;
	}

	/**
	 * Sets the default extended lookahead distance that is used when new paths are created.
	 * 
	 * @param distance - Specifies the extended lookahead distance in feet.
	 */
	public static void setDefaultExtendedLookAheadDistance(double distance) {
		k_extendedLookAheadDistance = distance;
	}

	/**
	 * Sets the curvature adjust value that is used when new paths are created.
	 *  The curvature adjust value is used to force the robot to more (or less) aggressively steer toward the
	 *  target point. A value of 1 will use the curvature that is computed by the Pure Pursuit algorithm.
	 *  A value greater than 1 will cause the robot to steer more aggressively towards that point and a 
	 *  value less than will will cause the robot to steer less aggressively.
	 * 
	 * @param curvatureAdjust - Specifies the adjustment value.
	 */
	public static void SetDefaultCurvatureAdjust(double curvatureAdjust) {
		k_curvatureAdjust = curvatureAdjust;
	}

	/**
	 * The Waypoint class defines one Bezier curve.
	 * 	The curves in question are Quintic bezier curves and as such require
	 *  six control points (each of which has an x and y component).
	 *	However because we require the first and second derivatives of joined curves to match,
	 *	the degrees of freedom are greatly reduced and the curve can be defined
	 *	by the end points, the starting and ending angles, and the lengths of the lines from
	 *  the end points and the control points. Note that only the starting point and starting
	 *  angles are specified in this class. The ending point and angle are defined by the next Waypoint in the list.
	 *  All distances are in feet
	 */
	public static class Waypoint
	{
	    public double x;			//!<Specifies the x coordinate of the starting point of the curve.
		public double y;			//!<Specifies the y coordinate of the starting point of the curve.
		public double angle;		//!<Specifies the angle (in <b>radians</b>) from the starting point
		public double  maxVelocity;	//!<Specifies the maximum velocity allowed  for this portion of the path. If this value is zero, then the global max velocity for the path is used.
	    public double l1 = 0;		//!<Specifies the distance to the first control point P1 using the starting angle.
	    public double l2 = 0;		//!<Specifies the distance to the second control point P2 using the starting angle.
	    public double l3 = 0;		//!<Specifies the distance to the third control point P3 measured from the next Waypoint's start position and using the next Waypoint's angle.
	    public double l4 = 0;		//!<Specifies the distance to the fourth control point P4 measured from the next Waypoint's start position and using the next Waypoint's angle.
	    
	    public Waypoint(double x_in, double y_in, double angle_in, double maxVelocity_in)
	    {
	    	x = x_in;
	    	y = y_in;
	    	angle = angle_in;
	    	maxVelocity = maxVelocity_in;
	    	l1 = 0;
	    	l2 = 0;
	    }
	    
	    public Waypoint(double x_in, double y_in, double angle_in)
	    {
	    	x = x_in;
	    	y = y_in;
	    	angle = angle_in;
	    	maxVelocity = 0;
	    	l1 = 0;
	    	l2 = 0;
	    }
	    
	    public Waypoint(double x_in, double y_in, double angle_in, double l1_in, double l2_in)
	    {
	    	x = x_in;
	    	y = y_in;
	    	angle = angle_in;
	    	maxVelocity = 0;
	    	l1 = l1_in;
	    	l2 = l2_in;
	    }
	    
	    public Waypoint(double x_in, double y_in, double angle_in, double l1_in, double l2_in, double maxVelocity_in)
	    {
	    	x = x_in;
	    	y = y_in;
	    	angle = angle_in;
	    	maxVelocity = maxVelocity_in;
	    	l1 = l1_in;
	    	l2 = l2_in;
	    }
	    
	    public Waypoint(double x_in, double y_in, double angle_in, double l1_in, double l2_in, double l3_in, double l4_in, double maxVelocity_in)
	    {
	    	x = x_in;
	    	y = y_in;
	    	angle = angle_in;
	    	maxVelocity = maxVelocity_in;
	    	l1 = l1_in;
	    	l2 = l2_in;
	    	l3 = l3_in;
	    	l4 = l4_in;
	    }

	}

	/**
	 * 
	 * The Segment class defines a segment of a path. Each segment
	 *  represents a point along the generated path specifying the current
	 *  position, velocity, acceleration and heading. An array of these segments
	 *  is generated for the left wheel, center of the robot and the right wheel.
	 * 
	 */
	public static class Segment
	{
		public double dt;			//!<Specifies the current time interval.
		public double x;			//!<Specifies the current absolute x position in feet.
		public double y;			//!<Specifies the current absolute y position in feet.
		public double position;		//!<Specifies the current linear distance in feet.
		public double velocity;		//!<specifies the current velocity in feet/second.
		public double acceleration;	//!<Specifies the current acceleration in feet/second*second.
		public double jerk;			//!<Specifies the current jerk int feet/second*second*second.
		public double heading;		//!<Specifies the current heading in radians.
	    
	    public Segment()
	    {
	    	dt = 0;
	    	x = 0;
	    	y = 0;
	    	position = 0;
	    	velocity = 0;
	    	acceleration = 0;
	    	jerk = 0;
	    	heading = 0;
	    }
	    
	    public Segment(Segment seg)
	    {
	    	dt = seg.dt;
	    	x = seg.x;
	    	y = seg.y;
	    	position = seg.position;
	    	velocity = seg.velocity;
	    	acceleration = seg.acceleration;
	    	jerk = seg.jerk;
	    	heading = seg.heading;
	    }
	    
	    public Segment(	double dt_in,
	    				double x_in,
	    				double y_in,
	    				double position_in,
	    				double acceleration_in,
	    				double jerk_in,
	    				double heading_in)
	    {
	    	dt = dt_in;
	    	x = x_in;
	    	y = y_in;
	    	position = position_in;
	    	acceleration = acceleration_in;
	    	jerk = jerk_in;
	    	heading = heading_in;
	    }
	    
	    // public static Segment[] CreateArray(int length)
	    // {
	    // 	Segment[] segments = new Segment[length];
	    	
	    // 	for (int i = 0 ; i < length ; i++)
	    // 	{
	    // 		segments[i]	= new Segment();
	    // 	}
	    	
	    // 	return(segments);
	    // }
	}
	
	private static class PathBezier
	{
		BezierQuintic[] m_bezierPoints;
		// SplinePoint[] m_splines;
		SplinePoint[] m_center;
		SplinePoint[] m_left;
		SplinePoint[] m_right;
		Segment[] m_centerSegments;
		Segment[] m_leftSegments;
		Segment[] m_rightSegments;
	}
	
	private static void computeDistance(SplinePoint[] trajectory, int idx)
	{
		if (idx > 0)
		{
			double dx	= trajectory[idx].m_x - trajectory[idx-1].m_x;
			double dy	= trajectory[idx].m_y - trajectory[idx-1].m_y;
			double d	= Math.sqrt(dx*dx + dy*dy);
			
			trajectory[idx].m_delta = d;
			trajectory[idx].m_distance = trajectory[idx-1].m_distance + d;
		}
	}
	
	private static void tankModify(SplinePoint[] original, SplinePoint[] left_trajectory, SplinePoint[] right_trajectory, int first, int count, double wheelbase_width) 
	{
	    double w = wheelbase_width / 2;
	    
	    for (int i = 0; i < count; i++) 
	    {
	        SplinePoint seg = original[i+first];
	        
	        double cos_angle = Math.cos(seg.m_heading);
	        double sin_angle = Math.sin(seg.m_heading);
	        	        
	        left_trajectory[i+first] = new SplinePoint(seg.m_x - (w * sin_angle), seg.m_y + (w * cos_angle), seg.m_heading, seg.m_maxVelocity);
	        right_trajectory[i+first] = new SplinePoint(seg.m_x + (w * sin_angle), seg.m_y - (w * cos_angle), seg.m_heading, seg.m_maxVelocity);
	        
	        computeDistance(left_trajectory, i+first);
	        computeDistance(right_trajectory, i+first);
	    }
	}
	
	@SuppressWarnings("unused")
	private static void printSplinePoints(SplinePoint[] points)
	{
		System.out.println("x,y,head,delta,dist");
        for (int j = 0 ; j < points.length ; j++)
        {
        	System.out.println(String.format("%f,%f,%f,%f,%f", points[j].m_x, points[j].m_y, 
        												MathUtil.normalizeDegrees(MathUtil.r2d(points[j].m_heading)),
        												points[j].m_delta, points[j].m_distance));
        }
		
	}
	
	private static int findPathPosition(SplinePoint[] points, double position, int idx)
	{
		while ((idx < points.length-1) && (position > points[idx].m_distance))
		{
			idx++;
		}
		
		return(idx);
	}
	
	private static int findPathPositionRev(SplinePoint[] points, double position, int idx)
	{
		while ((idx > 0) && (position < points[idx].m_distance))
		{
			idx--;
		}
		
		return(idx);
	}
	
	private static void addSegment(SplinePoint[] points, int idx, int lastIdx, double deltaPos, Segment[] segments, int segIdx, double dt, double d, boolean reverse, double startVelocity)
	{
		double x = points[lastIdx].m_x + (points[idx].m_x - points[lastIdx].m_x) * deltaPos;
		double y = points[lastIdx].m_y + (points[idx].m_y - points[lastIdx].m_y) * deltaPos;
		
		Segment segment = new Segment();
		
		segment.dt = dt;
		segment.x = x;
		segment.y = y;
		segment.heading = points[idx].m_heading;
		segment.velocity = d / dt;
		
		double dx;
		double dy;
		
		if (reverse)
		{
			dx = (segIdx < segments.length-1) ? segments[segIdx + 1].x - x : points[points.length-1].m_x - x;
			dy = (segIdx < segments.length-1) ? segments[segIdx + 1].y - y : points[points.length-1].m_y - y;
		}
		else
		{
			dx = (segIdx > 0) ? x - segments[segIdx - 1].x : x - points[0].m_x;
			dy = (segIdx > 0) ? y - segments[segIdx - 1].y : y - points[0].m_y;
		}
		double angle = Math.atan2(dy, dx);
		double da = MathUtil.normalizeRadians(angle - segment.heading);
		
		if (Math.abs(da) > Math.PI/2)
		{
			segment.velocity = -segment.velocity;
		}
		
		double dv;
		
		if (reverse)
		{
			if (segIdx < segments.length-1)
			{
				segment.position = segments[segIdx + 1].position - d;
				dv = segment.velocity - segments[segIdx + 1].velocity;
			}
			else
			{
				segment.position = points[points.length-1].m_distance - d;
				dv = segment.velocity - startVelocity;
			}
		}
		else
		{
			segment.position = (segIdx > 0) ? segments[segIdx - 1].position + d : d;
			dv = (segIdx > 0) ? segment.velocity - segments[segIdx - 1].velocity : segment.velocity;
		}
		
		segment.acceleration = dv / dt;
		
		if (reverse)
		{
			segment.acceleration = -segment.acceleration;
		}
		
		segments[segIdx]	= segment;
	}
	
	
	private static int computePath( SplinePoint[] left,
									SplinePoint[] center,
									SplinePoint[] right,
									Segment[] leftSeg,
									Segment[] centerSeg,
									Segment[] rightSeg,
									double dt,
									double max_velocity,
									double max_acceleration,
									double max_jerk,
									boolean reverse,
									double endPosition,
									double startVelocity)
	{
		int segIdx = reverse ? centerSeg.length - 1 : 0;
		double position = reverse ? center[center.length-1].m_distance : 0;
		double acceleration = 0;
		double velocity = startVelocity;
		int lastIdx = reverse ? center.length - 1 : 0;
		
		max_velocity = center[lastIdx].m_maxVelocity;

		double jerkSpeed = max_jerk != 0 ? max_velocity - 0.5*(max_acceleration * max_acceleration / max_jerk) : 0;
		
		while (reverse ? (lastIdx > 0) && (velocity < max_velocity) : (lastIdx < center.length-1) && (position < endPosition))
		{
			double nextVelocity = velocity;
			
			if (max_jerk > 0)
			{
				if (center[lastIdx].m_maxVelocity > max_velocity)
				{
					max_velocity = center[lastIdx].m_maxVelocity;
					jerkSpeed = max_velocity - 0.5*(max_acceleration * max_acceleration / max_jerk);
				}
				else if (center[lastIdx].m_maxVelocity < max_velocity)
				{
					max_velocity = center[lastIdx].m_maxVelocity;
					jerkSpeed = max_velocity + 0.5*(max_acceleration * max_acceleration / max_jerk);
				}
			}
			
			if (jerkSpeed != 0)
			{
				if (nextVelocity < max_velocity)
				{
					if (velocity >= jerkSpeed)
					{
						acceleration -= dt * max_jerk;
						
						if (acceleration < 0)
						{
							acceleration = 0;
							nextVelocity = max_velocity;
						}
						else
						{
							nextVelocity += dt * acceleration;
						}
					}
					else 
					{
						acceleration += dt * max_jerk;
						
						if (acceleration > max_acceleration)
						{
							acceleration = max_acceleration;
						}
						
						nextVelocity += dt * acceleration;
					}
					
					if (nextVelocity > max_velocity)
					{
						nextVelocity = max_velocity;
					}
				}
				else if (nextVelocity > max_velocity)
				{
//					if (nextVelocity < max_velocity)
					{
						if (velocity <= jerkSpeed)
						{
							acceleration += dt * max_jerk;
							
							if (acceleration > 0)
							{
								acceleration = 0;
								nextVelocity = max_velocity;
							}
							else
							{
								nextVelocity += dt * acceleration;
							}
						}
						else 
						{
							acceleration -= dt * max_jerk;
							
							if (acceleration < -max_acceleration)
							{
								acceleration = -max_acceleration;
							}
							
							nextVelocity += dt * acceleration;
						}
						
						if (nextVelocity < max_velocity)
						{
							nextVelocity = max_velocity;
						}
					}
				}
			}
			else
			{
				if (nextVelocity < max_velocity)
				{
					nextVelocity += dt * max_acceleration;
					
					if (nextVelocity > max_velocity)
					{
						nextVelocity = max_velocity;
					}

				}
				else if (nextVelocity > max_velocity)
				{
					nextVelocity -= dt * max_acceleration;
					
					if (nextVelocity < max_velocity)
					{
						nextVelocity = max_velocity;
					}
				}
			}
			
			
			double d = (dt * (velocity + nextVelocity) / 2);
			
			int idx;
			
			if (reverse)
			{
				idx = findPathPositionRev(center, position - d, lastIdx);
			}
			else
			{
				idx = findPathPosition(center, position + d, lastIdx);
			}
				
			if (idx == lastIdx)
			{
				if (reverse)
				{
					lastIdx++;
				}
				else
				{
					lastIdx--;
				}
			}
			
			double D = center[idx].m_distance - center[lastIdx].m_distance;
			double DL = left[idx].m_distance - left[lastIdx].m_distance;
			double DR = right[idx].m_distance - right[lastIdx].m_distance;
			double dl = d * DL / D;		// Distance left wheel moved
			double dr = d * DR / D;		// Distance right wheel moved
						
			/*
			 * If the left or right has moved too far, shorten the center to keep it inline
			 */
			if (dl > d)
			{
				dl = d;
				d  = dl * D / DL;
				dr = dl * DR / DL;
			}
			else if (dr > d)
			{
				dr = d;
				d  = dr * D / DR;
				dl = dr * DL / DR;
			}
			
			if (reverse)
			{
				position -= d;
			}
			else
			{
				position += d;
			}
			
			/*
			 * Compute percentage
			 */
			double deltaPos = (position - center[lastIdx].m_distance) / D;

			
			if ((segIdx < 0) || (segIdx > centerSeg.length))
			{
				throw new java.lang.ArrayIndexOutOfBoundsException("Insufficient segment space");
			}
			
			/*
			 * Create segments
			 */
			addSegment(center, idx, lastIdx, deltaPos, centerSeg, segIdx, dt, d, reverse, startVelocity);
			addSegment(left, idx, lastIdx, deltaPos, leftSeg, segIdx, dt, dl, reverse, startVelocity);
			addSegment(right, idx, lastIdx, deltaPos, rightSeg, segIdx, dt, dr, reverse, startVelocity);
			
			if (reverse)
			{
				segIdx--;
			}
			else
			{
				segIdx++;
			}
			velocity = nextVelocity;
			lastIdx = idx;

			if (reverse)
			{
				if (velocity == max_velocity)
				{
					break;
				}
			}
		}
		
//		printSegments(centerSeg, leftSeg, rightSeg, segIdx + 1);
		
		return(segIdx);
	}	
	
	private static void followPath(PathBezier path, double velocity, double dt, double max_velocity, 
																	double max_acceleration, double max_decl, double max_jerk, double finalVelocity)
	{
		/*
		 * Make a rough guess of how many segments we will need
		 */
		double length = path.m_center[path.m_center.length-1].m_distance;
		int count = (int) (5 * length / (max_velocity * dt)) + 1000;
		
		path.m_centerSegments = new Segment[count];
		path.m_leftSegments = new Segment[count];
		path.m_rightSegments = new Segment[count];
		
		/*
		 * First compute the ending deceleration segments
		 */
		Segment[] endCenter = new Segment[count];
		Segment[] endLeft = new Segment[count];
		Segment[] endRight = new Segment[count];
		
		int endSegIdx = computePath( 	path.m_left,
										path.m_center,
										path.m_right,
										endLeft,
										endCenter,
										endRight,
										dt,
										max_velocity,
										max_decl,
										max_jerk,
										true,
										0,
										finalVelocity) + 1;
		
		/*
		 * Now compute the starting segments up to the start of the deceleration period
		 */
		int segIdx = computePath( 	path.m_left,
									path.m_center,
									path.m_right,
									path.m_leftSegments,
									path.m_centerSegments,
									path.m_rightSegments,
									dt,
									max_velocity,
									max_acceleration,
									max_jerk,
									false,
									endCenter[endSegIdx].position,
									0);
		
		/*
		 * Now paste together the two sequences
		 */
		for (int s = endSegIdx ; s < endCenter.length ; s++, segIdx++)
		{
			path.m_centerSegments[segIdx] = endCenter[s];
			path.m_leftSegments[segIdx] = endLeft[s];
			path.m_rightSegments[segIdx] = endRight[s];
		}
		
		path.m_centerSegments = java.util.Arrays.copyOf(path.m_centerSegments, segIdx);
		path.m_leftSegments = java.util.Arrays.copyOf(path.m_leftSegments, segIdx);
		path.m_rightSegments = java.util.Arrays.copyOf(path.m_rightSegments, segIdx);
	}
	
	private static void printSegments(Segment[] center, Segment[] left, Segment[] right, int start)
	{
		System.out.println("t,dt,cx,cy,cp,dcp,cv,ca,lx,ly,lp,dlp,lv,la,rx,ry,rp,drp,rv,ra");
		
		double time = 0;
		
		for (int i = start ; (i < center.length) ; i++)
		{
			time += center[i].dt;
			
			double dcp = (i > start) ? center[i].position - center[i-1].position : 0;
			double dlp = (i > start) ? left[i].position - left[i-1].position : 0;
			double drp = (i > start) ? right[i].position - right[i-1].position : 0;
			
			System.out.println(String.format("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", 
												time, center[i].dt, 
												center[i].x, center[i].y, center[i].position, dcp, center[i].velocity, center[i].acceleration,
												left[i].x, left[i].y, left[i].position, dlp, left[i].velocity, left[i].acceleration,
												right[i].x, right[i].y, right[i].position, drp, right[i].velocity, right[i].acceleration));
		}
	}

	/**
	 * Prints the specified path to the console. Useful for debugging.
	 * 
	 * @param path - Specifies the path to print.
	 */
	public static void printPath(Path path)
	{
		printSegments(path.m_centerPath, path.m_leftPath, path.m_rightPath, 0);
	}
	
	public static class Path
	{
		/*
		 * These are the parameters to be used by PurePursuit for this path.
		 *   They are initialized from the default values above but can
		 *   be altered before calling loadPath. Note that if these values
		 *   are changed after calling loadPath, they will be ignored until
		 *   the next time loadPath is called
		 */
		//! @cond PRIVATE 
		public double m_lookAheadTime = k_lookAheadTime;			// Time along the path of the next target point	
		public double m_minLookAheadDist = k_minLookAheadDist;		// Minimum allowed look ahead distance (overrides look ahead time if necessary)
		public double m_maxSearchTime = k_maxSearchTime;		// Max time along the path to search for the closest point to the current position
		public double m_minSpeed = k_minSpeed;						// Minimum allowed speed
		public double m_extendedLookAheadDistance = k_extendedLookAheadDistance;	// Distance to extend path if m_isExtended is true
		public double m_curvatureAdjust = k_curvatureAdjust;		// Adjusts the curvature to make the path following more or less aggressive
																	//   A value of 1 is normal, a value greater than 1 makes the 
																	//   path following more aggressive, less than 1 makes it less aggressive
		//! @endcond

		/** 
		 * This function overrides the <b>lookAheadTime</b>.
		 * 	The <b>lookAheadTime</b> is used to find a point in the future along the path
		 *  to which the robot is steered.
		 *  If this function is not called then the default <b>lookAheadTime</b> as set in
		 *  the Pathfinder class will be used.
		 * 
		 *  @param lookAheadTime - Specifies the new lookahead time.
		 */
		public void setLookAheadTime(double lookAheadTime)
		{
			m_lookAheadTime = lookAheadTime;
		}

		/** 
		 * This function overrides the <b>minLookAheadDist</b>.
		 * 	If the distance to the point found by using the <b>lookAheadTime</b> is less than
		 *  the <b>minLookAheadDist</b>, then a new target point is found with at least
		 *  this minimum distance.
		 *  If this function is not called then the default <b>minLookAheadDist</b> as set in
		 *  the Pathfinder class will be used.
		 * 
		 *  @param minLookAheadDist - Specifies the new minimum lookahead distance.
		 */
		public void setMinLookAheadDist(double minLookAheadDist) {
			m_minLookAheadDist = minLookAheadDist;
		}

		/** 
		 * This function overrides the <b>maxSearchTime</b>.
		 * 	This is the maximum time to look ahead on the path to find the closest point
		 *  to the current robot position.
		 *  If this function is not called then the default <b>maxSearchTime</b> as set in
		 *  the Pathfinder class will be used.
		 * 
		 *  @param maxSearchTime - Specifies the new search time.
		 */
		public void setMaxSearchTime(double maxSearchTime) {
			m_maxSearchTime = maxSearchTime;
		}

		/** 
		 * This function overrides the <b>minSpeed</b>.
		 * 	This is the minimum speed for the robot which keeps the robot from stalling.
		 *  If this function is not called then the default <b>minSpeed</b> as set in
		 *  the Pathfinder class will be used.
		 * 
		 *  @param minSpeed - Specifies the new minimum speed.
		 */
		public void setMinSpeed(double minSpeed)
		{
			m_minSpeed = minSpeed;
		}

		/** 
		 * This function overrides the <b>extendedLookAheadDistance</b>.
		 * 	If extended lookahead is enabled then this is the distance to extend
		 *  the path beyond the end for the purpose of navigation. Note the robot will
		 *  still stop at the original end position.
		 *  If this function is not called then the default <b>extendedLookAheadDistance</b> as set in
		 *  the Pathfinder class will be used.
		 * 
		 *  @param extendedLookAheadDistance - Specifies the new distance.
		 */
		public void setExtendedLookAheadDistance(double extendedLookAheadDistance) {
			m_extendedLookAheadDistance = extendedLookAheadDistance;
		}

		/** 
		 * This function overrides the <b>curvatureAdjust</b>.
		 *  The curvature adjust value is used to force the robot to more (or less) aggressively steer toward the
		 *  target point. A value of 1 will use the curvature that is computed by the Pure Pursuit algorithm.
		 *  A value greater than 1 will cause the robot to steer more aggressively towards that point and a 
		 *  value less than will will cause the robot to steer less aggressively.
		 *  If this function is not called then the default <b>curvatureAdjust</b> as set in
		 *  the Pathfinder class will be used.
		 * 
		 *  @param curvatureAdjust - Specifies the new curvature adjustment.
		 */
		public void setCurvatureAdjust(double curvatureAdjust) {
			m_curvatureAdjust = curvatureAdjust;
		}

		//! @cond PRIVATE 
		public double m_dt;
		public double m_wheelBase;
		public BezierQuintic[] m_bezierPoints;
		public Segment[] m_centerPath;
		public Segment[] m_leftPath;
		public Segment[] m_rightPath;
		//! @endcond
		
		private Path(BezierQuintic[] bezierPoints, Segment[] center, Segment[] left, Segment[] right, double dt, double wheelBase)
		{
			m_bezierPoints = bezierPoints;
			m_centerPath = center;
			m_leftPath = left;
			m_rightPath = right;
			m_dt = dt;
			m_wheelBase = wheelBase;
		}
	}
	
	@SuppressWarnings("unused")
	private static void printSplinePoints(SplinePoint[] left, SplinePoint[] center, SplinePoint[] right)
	{
		System.out.println("lx,ly,ld,lp,cx,cy,cd,cp,rx,ry,rd,rp,a");
		
		for (int i = 0 ; i < center.length ; i++)
		{
			System.out.println(String.format("%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f,%f", 
								left[i].m_x, left[i].m_y, left[i].m_delta, left[i].m_distance,
								center[i].m_x, center[i].m_y, center[i].m_delta, center[i].m_distance,
								right[i].m_x, right[i].m_y, right[i].m_delta, right[i].m_distance,
								MathUtil.r2d(center[i].m_heading)));
		}
	}
		
	// public static Path computePath(	final Waypoint[] path_in, 
	// 		int sample_count, 
	// 		double dt,
	// 		double max_velocity, 
	// 		double max_acceleration, 
	// 		double max_jerk,
	// 		double wheelBase)
	// {
	// 	return(computePath(path_in, sample_count, dt, max_velocity, max_acceleration, max_acceleration, max_jerk, wheelBase));
	// }
	
	/**
	 * Computes a path from the specified waypoints.
	 * 
	 * @param waypoints_in - Specifies the waypoints which define the path.
	 * @param sample_count - Specifies the number of points/bezier curve to generate.
	 * @param dt - Specifies the time between points on the output path.
	 * @param max_velocity - Specifies the maximum allowable velocity for either of the wheels.
	 * @param max_acceleration - Specifies the maximum allowable acceleration.
	 * @param max_decl - Specifies the maximum allowable deceleration.
	 * @param wheelbase - Specifies the width of the wheelbase in feet.
	 */
	public static Path computePath(	final Waypoint[] waypoints_in, 
									int sample_count, 
									double dt,
									double max_velocity, 
									double max_acceleration, 
									double max_decl,
									double max_jerk,
									double wheelBase)
	{
	    if (waypoints_in.length < 2) return(null);
	    
	    Waypoint[] waypoints = new Waypoint[waypoints_in.length];
	    
	    for (int i = 0 ; i < waypoints_in.length ; i++)
	    {
	    	waypoints[i] = new Waypoint(waypoints_in[i].x, waypoints_in[i].y, waypoints_in[i].angle, waypoints_in[i].l1, waypoints_in[i].l2, waypoints_in[i].l3, waypoints_in[i].l4, waypoints_in[i].maxVelocity > 0 ? waypoints_in[i].maxVelocity : max_velocity);
	    }
	    
	    PathBezier pathSpline = new PathBezier();
	    
	    SplinePoint[] centerSpline = new SplinePoint[(waypoints.length-1) * (sample_count)];
	    SplinePoint[] leftSpline = new SplinePoint[centerSpline.length];
	    SplinePoint[] rightSpline = new SplinePoint[centerSpline.length];
	    
	    pathSpline.m_center = centerSpline;
	    pathSpline.m_left = leftSpline;
	    pathSpline.m_right = rightSpline;
	    pathSpline.m_bezierPoints = new BezierQuintic[waypoints.length - 1];
	    
	    for (int i = 0 ; i < waypoints.length - 1 ; i++) 
	    {
        	BezierQuintic bezier = new BezierQuintic(waypoints[i].x, waypoints[i].y, waypoints[i].angle, waypoints[i].l1, waypoints[i].l3,
        								waypoints[i+1].x, waypoints[i+1].y, waypoints[i+1].angle + Math.PI, waypoints[i].l2, waypoints[i].l4);
        	
        	bezier.ComputeSplinePoints(centerSpline,  i * (sample_count), sample_count, waypoints[i].maxVelocity);
        	
        	pathSpline.m_bezierPoints[i] = bezier;
	        
	    	tankModify(centerSpline, leftSpline, rightSpline, i * (sample_count), sample_count, wheelBase); 
	    	

	    }
	    
//	      printSplinePoints(leftSpline, centerSpline, rightSpline);
//        printSplinePoints(centerSpline);
//        printSplinePoints(leftSpline);
//        printSplinePoints(rightSpline);
        
    	followPath(pathSpline, 0, dt, max_velocity, max_acceleration, max_decl, max_jerk, waypoints_in[waypoints_in.length-1].maxVelocity);
//    	fixupPath(pathSpline, dt, max_acceleration, wheelBase);
    	
//    	printSegments(pathSpline.m_centerSegments, pathSpline.m_leftSegments, pathSpline.m_rightSegments, 0);
    	
    	return(new Path(	pathSpline.m_bezierPoints, 
							pathSpline.m_centerSegments, 
							pathSpline.m_leftSegments, 
							pathSpline.m_rightSegments,
							dt,
							wheelBase));
	}
}
