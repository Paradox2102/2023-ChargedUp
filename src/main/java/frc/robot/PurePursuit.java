package frc.robot;

import frc.ApriltagsCamera.Logger;
import frc.lib.CSVWriter;

// import java.util.function.DoubleSupplier;

import frc.lib.CSVWriter.Field;
import frc.robot.LocationTracker;
import frc.robot.Navigator.NavigatorPos;

import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.function.DoubleSupplier;

import javax.tools.DocumentationTool.Location;

import edu.wpi.first.wpilibj.XboxController;
import frc.pathfinder.Pathfinder.Path;
import frc.pathfinder.Pathfinder.Segment;

/**
 * 
 * @brief The PurePursuit class provides path following using the Pure Pursuit algorithm.
 * 
 * 			This algorithm is described here: https://www.ri.cmu.edu/pub_files/pub3/coulter_r_craig_1992_1/coulter_r_craig_1992_1.pdf
 * 			It takes as input paths generated by the Pathfinder class. There are a number
 * 			of adjustable parameters that can be set when creating the paths that will
 * 			control the behavior of the Pure Pursuit. See the Pathfinder class for details.
 *
 */
public class PurePursuit {

	/**
	 * 
	 * @brief SetSpeed defines the interface used by PurPursuit to set the speed of the motors.
	 * 			The speed should be set in feet/second
	 */

	public interface SetSpeed
	{
		/**
		 * Sets the motor speeds
		 * 
		 * @param left - Sets the speed of the left motor in feet/second.
		 * @param right - Sets the speed of the right motor in feet/second.
		 */
		void set(double left, double right);
	}

	// Configurable parameters
	private double k_lookAheadTime = 0.75;		// Time along the path of the next target point	
	private double k_minLookAheadDist = 0.6;	// Minimum allowed look ahead distance (overrides look ahead time if necessary)
	private double k_maxSearchTime = 1;		// Max time along the path to search for the closest point to the current position
	private double k_minSpeed = 0.25;			// Minimum allowed speed
	private double k_curvatureAdjust = 1.1;		// Adjusts the curvature to make the path following more or less aggressive
												//   A value of 1 is normal, a value greater than 1 makes the 
												//   path following more aggressive, less than 1 makes it less aggressive
	private double k_extendedLookAhead = .75;	// Distance to extend path if m_isExtended is true
	private int m_rate = 50;					// Thread update rate in milliseconds

	private final Object m_lockWriter = new Object();	// Lock for logging
	private final Object m_dataLock = new Object();		// Lock for data

	private AtomicBoolean m_finished = new AtomicBoolean(false);	// True if path has finished
	private AtomicBoolean m_runThread = new AtomicBoolean(false);	// If true, follow path
	private AtomicInteger m_prevIdx = new AtomicInteger(0);

	private double m_dt = 0.02;				// Time increment for points on path
	private double m_wheelBase = 0.75;		// Width of the wheelbase

	private int k_lookAheadIdx = (int) (k_lookAheadTime / m_dt);		// Index increment to reach specified time							
	private int k_maxLookAhead = (int) (k_maxSearchTime / m_dt);	// Max index count to look ahead for closest point to current position

	// private final int k_checkStall = 40;			// Distance from the end of the path to check for stall - MUSTFIX - with min speed is this really necessary?

	private Path m_loadedPath;						// Current loaded path;
	private Path m_path;							// Current path data
	private boolean m_isReversed;					// If true, the robot is to drive backwards
	private boolean m_isExtended;					// If true, for the curvature calculation, the path is extended by k_extendedLookAhead tangent to the ending point
	private boolean m_setPosition;					// If true, set the position and angle from the first point on the path.

	private Thread m_thread = null;					// Thread for processing path
	private long m_nextRun;							// Time for the next curvature calculation

	private Navigator m_navigator;					// Navigator which gives robot position data
	private SetSpeed m_setSpeed;					// Function to call to set the speed for the left and right motors in FPS

	private DoubleSupplier m_stickX;                // Gets Joystick X
	private final double k_maxSpeed = 0;            // Max Speed of the robot

	/*
	 * Logging fields
	 */
	private Field[] k_fields = { new Field("Yaw", 'f'), new Field("Vel", 'f'), new Field("Ideal Vel L", 'f'), new Field("Ideal Vel R", 'f'),
			new Field("Vel L", 'f'), new Field("Vel R", 'f'), new Field("Cur X", 'f'), new Field("Cur Y", 'f'),
			new Field("Ideal x", 'f'), new Field("Ideal y", 'f'), new Field("Next Pos x", 'f'),
			new Field("Next Pos y", 'f'), new Field("L", 'f'), new Field("dX", 'f'), new Field("Theta", 'f'),
			new Field("Curvature", 'f'), new Field("velDif", 'f'),
			new Field("Closest idx", 'd'), new Field("left pos", 'd'), new Field("right pos", 'd') }; 
			// new Field("Update Count", 'd'), new Field("Errors", 'd') };

	private CSVWriter m_writer;		// Used for logging

	/** 
	 * @param navigator - Specifies the Navigator class to be use to obtain the robot's position.
	 * @param setSpeed - Specifies the callback function used to set the robot's speed. This function should accept the speed in feet/second.
	 * @param rate - Specifies the time between updates in milliseconds
	 */
	public PurePursuit(Navigator navigator, SetSpeed setSpeed, int rate, DoubleSupplier stickX) {
		m_navigator = navigator;
		m_setSpeed = setSpeed;
		m_rate = rate;
		m_stickX = stickX;
	}

		/**
		 * Loads the current path
		 * 
		 * @param path - Specifies the path to follow.
		 * @param isReversed - If true, the robot backs along the path.
		 * @param isExtended - If true, the path is extended beyond the end, tangent to the ending angle, for the purpose of tracking (the robot will still stop when the original end is reached.)
		 * @param setPosition - If true, set the starting position and angle based on the first point of the path.
		 */
	public void loadPath(Path path, boolean isReversed, boolean isExtended, boolean setPosition) {
		synchronized (m_dataLock) {
			stopFollow();		// just in case

			m_loadedPath = path;
			m_isReversed = isReversed;
			m_isExtended = isExtended;
			m_setPosition = setPosition;

			m_dt = path.m_dt;
			m_wheelBase = path.m_wheelBase;

			k_lookAheadTime = path.m_lookAheadTime;
			k_minLookAheadDist = path.m_minLookAheadDist;
			k_maxSearchTime = path.m_maxSearchTime;
			k_minSpeed = path.m_minSpeed;
			k_curvatureAdjust = path.m_curvatureAdjust;
			k_extendedLookAhead = path.m_extendedLookAheadDistance;

			k_lookAheadIdx = (int) (k_lookAheadTime / m_dt);	// Index increment to reach specified time							
			k_maxLookAhead = (int) (k_maxSearchTime / m_dt);	// Max index count to look ahead for closest point to current position		
		}
	}

	/**
	 * Enables logging of the robots motion along the path to the robot's local file system.
	 * 
	 * @param logPath - Specifies the path directory into which the logs are stored
	 */
	public void enableLogging(String logPath)
	{
		synchronized (m_lockWriter)
		{
			if (m_writer != null)
			{
				m_writer.finish();
			}

			m_writer = new CSVWriter(logPath, "Follow Profile", k_fields);
		}
	}

	/**
	 * Disables logging of the robots motion.
	 * 
	 */
	public void disableLogging()
	{
		synchronized (m_lockWriter)
		{
			if (m_writer != null)
			{
				m_writer.finish();
				m_writer = null;
			}
		}
	}

	private void startLogging()
	{
		synchronized (m_lockWriter)
		{
			if (m_writer != null)
			{
				m_writer.start();
			}
		}
	}

	private void finishLogging()
	{
		synchronized (m_lockWriter)
		{
			if (m_writer != null)
			{
				m_writer.finish();
			}
		}
	}

	private void logData(Object... values)
	{
		synchronized (m_lockWriter)
		{
			if (m_writer != null)
			{
				m_writer.write(values);
			}
		}
	}

	/*
	 * Path following computation thread
	 */
	private void startThread() {
		if (m_thread == null) {
			
			(m_thread = new Thread() {
				public void run()
				{
					m_nextRun = System.currentTimeMillis() + m_rate;

					while (true)
					{
						if (m_runThread.get()) {
							SpeedContainer speedContainer = followPath();
				
							m_setSpeed.set(speedContainer.leftSpeed, speedContainer.rightSpeed);
						}
						
						/*
						 * Sleep for remaining time and calculate next time to run
						 */
						try {
							long sleepTime = m_nextRun - System.currentTimeMillis();
							m_nextRun += m_rate;
							
							if (sleepTime > 0) {
								sleep(sleepTime);
							}
						} catch (final InterruptedException e) {
							e.printStackTrace();
						}
					}
				}
			}).start();
		}

		m_runThread.set(true);
	}

	void stopThread()
	{
		m_runThread.set(false);
	}

	/**
	 * Start following the currently loaded path.
	 * While the path is being followed, PurePursuit must have exclusive
	 * access to the drive motors.
	 * 
	 */
	public void startPath() {
		m_finished.set(false);		// Path is not finished

		if (m_setPosition)
		{
			Segment first = m_loadedPath.m_centerPath[0];
			double angle = Math.toDegrees(first.heading);

			if (m_isReversed)
			{
				angle += 180;
			}

			Logger.log("PurePursuit", 1, String.format("set: angle=%f, x=%f, y=%f", angle, first.x, first.y));

			m_navigator.reset(angle, first.x, first.y);
		}
		startLogging();				// Start logging if required
		startThread();				// Start and/or enable the processing thread
	}

	/**
	 * @return Returns true if the path is complete
	 * 
	 */
	public boolean isFinished() {
		return m_finished.get();
	}

	/**
	 * Stop following the path.
	 * 
	 */
	public void stopFollow() {
		finishLogging();			// End logging if necessary
		
		m_finished.set(true);		// Signal that the path is complete
		m_runThread.set(false);		// Disable the processing thread
		m_prevIdx.set(0);			// MUSTFIX - should move this  to startPath() ?
	}

	/*
	 * The path can optionally end with a non-zero velocity. This function retrieves that velocity.
	 */
	private SpeedContainer getEndingVel() {
		double leftVel = m_path.m_leftPath[m_path.m_leftPath.length - 1].velocity;
		double rightVel = m_path.m_rightPath[m_path.m_rightPath.length - 1].velocity;

		if (m_isReversed) {
			leftVel *= -1;
			rightVel *= -1;
		}

		return new SpeedContainer(leftVel, rightVel);
	}

	/*
	 * This function computes the left and right motor speeds required to follow the path
	 */
	private SpeedContainer followPath() {
		NavigatorPos pos = m_navigator.getPos();

		synchronized (m_dataLock)
		{
			m_path = m_loadedPath;
		}
		
		if (m_finished.get()) {
			return getEndingVel();
		}

		// Find the index of the closest point on the path to the robot's current position
		int closestPoint = getClosestPoint(pos);
		
		Segment closestPos = m_path.m_centerPath[closestPoint];

		m_prevIdx.set(closestPoint);	// Save this index for next time

		// See if we are at the end of the path
		if (closestPoint == m_path.m_centerPath.length - 1) {
			stopFollow();
			return getEndingVel();
		}

		// Look forward on the path by the time indicated
		int lookAheadIdx = closestPoint + k_lookAheadIdx;

		if (lookAheadIdx >= m_path.m_centerPath.length) {
			lookAheadIdx = m_path.m_centerPath.length - 1;
		}

		// MUSTFIX - really need this?
		// if (closestPathIdx > m_path.m_centerPath.length - k_checkStall && !m_isReversed) {// was 50
		// 	if (Math.abs(pos.leftSpeed) <= 0.1 && Math.abs(pos.rightSpeed) <= 0.1) {
		// 		stopFollow();
		// 		Logger.Log("PurePursuit", 9, "Stalled");
		// 		return new SpeedContainer(0, 0);
		// 	}
		// }

		Segment nextPos = m_path.m_centerPath[lookAheadIdx];
		double velocity;
		if (m_stickX != null) {
			velocity = closestPos.velocity;
		} else {
			double x = m_stickX.getAsDouble();
			// how many times is this called? ^^^
			velocity = k_maxSpeed * x;
		}


		if (velocity < k_minSpeed) {
			velocity = k_minSpeed;
		}

		if (m_isReversed) {
			velocity = -velocity;
		}

		double distance = calcDistance(pos.x, pos.y, nextPos);

		if (distance < k_minLookAheadDist) {
			int nextPosIdx = getLookAheadPoint(pos, k_minLookAheadDist, closestPoint);
			
			if (nextPosIdx < 0 && m_isExtended) {
				/*
				 * The next point is at the end of the path and extended is enabled.
				 *   Compute a new point that is k_extendedLookAhead beyond the end of
				 *   the path, tangent to the end of the path
				 */
				Segment newPos = m_path.m_centerPath[m_path.m_centerPath.length - 1];
				double newX = newPos.x + k_extendedLookAhead * Math.cos(m_path.m_centerPath[m_path.m_centerPath.length - 1].heading);
				double newY = newPos.y + k_extendedLookAhead * Math.sin(m_path.m_centerPath[m_path.m_centerPath.length - 1].heading);
				
				nextPos = new Segment(0, newX, newY, 0, 0, 0, 0);	// dummy segment for this new position
			} else if (nextPosIdx < 0) {
				/*
				 * The next point is beyond the end of the path and the path is not extended 
				 *   so use the end of the path
				 */
				nextPos = m_path.m_centerPath[m_path.m_centerPath.length - 1];
			} else {
				/*
				 * Use the next point
				 */
				nextPos = m_path.m_centerPath[nextPosIdx];
			}
			distance = calcDistance(pos.x, pos.y, nextPos);	// calculate the new distance
		}

		// Compute the 'curvature'
		double angle = pos.yaw;
		double theta = Math.atan2(nextPos.y - pos.y, nextPos.x - pos.x);
		double dX = distance * Math.sin(theta - Math.toRadians(!m_isReversed ? angle : angle + 180));
		double curvature = (2 * dX) / (distance * distance);

		// Compute the required left and right motor speeds
		double speedDiff = velocity * curvature * (m_wheelBase / 2.0) * k_curvatureAdjust;
		double leftSpeed = !m_isReversed ? velocity - speedDiff : velocity + speedDiff;
		double rightSpeed = !m_isReversed ? velocity + speedDiff : velocity - speedDiff;

		// logData(pos.yaw, velocity, (leftSpeed), (rightSpeed), pos.leftSpeed, pos.rightSpeed, pos.x,
		// 		pos.y, closestPos.x, closestPos.y, nextPos.x, nextPos.y, distance, dX, theta, curvature, speedDiff,
		// 		closestPoint, pos.leftPos, pos.rightPos);	//, pos.updateCount, pos.errorCount);

		return new SpeedContainer(leftSpeed, rightSpeed);
	}

	// private class ClosestPoint
	// {
	// 	public final double distance;
	// 	public final int index;

	// 	public ClosestPoint(double distance, int index)
	// 	{
	// 		this.distance = distance;
	// 		this.index = index;
	// 	}
	// }
	/*
	 * This function finds the closest point on the path to the current robot position
	 *   It starts searching at the last closest point and will search a maximum of k_lookAheadPoints
	 * 
	 */
	private int getClosestPoint(NavigatorPos pos) {
		double smallestDistance;
		int closestIdx;

		int maxLookAhead = m_prevIdx.get() + k_maxLookAhead;

		if (maxLookAhead > m_path.m_centerPath.length) {
			maxLookAhead = m_path.m_centerPath.length;
		}

		closestIdx = maxLookAhead - 1;
		smallestDistance = calcDistance(pos.x, pos.y, m_path.m_centerPath[closestIdx]);

		int prevIdx = m_prevIdx.get();
		for (int i = prevIdx; i < maxLookAhead; i++) {
			double distance = calcDistance(pos.x, pos.y, m_path.m_centerPath[i]);
			if (distance < smallestDistance) {
				smallestDistance = distance;
				closestIdx = i;
			}
		}

		return closestIdx;	//new ClosestPoint(smallestDistance, closestIdx);
	}

	/*
	 * This function finds the next point on the path that is lookAheadFt distance from the current robot position
	 *   It starts looking at the closest path point to the current robot position and continues until
	 *   it finds a point that is the correct distance or until the end of the search is reached
	 */
	private int getLookAheadPoint(NavigatorPos pos, double lookAheadFt, int closestPoint) {
		int lookAheadPoints = closestPoint + k_maxLookAhead;
		
		if (lookAheadPoints > m_path.m_centerPath.length) {
			lookAheadPoints = m_path.m_centerPath.length;
		}

		for (int i = closestPoint; i < lookAheadPoints; i++) {
			if (calcDistance(pos.x, pos.y, m_path.m_centerPath[i]) >= lookAheadFt) {
				return i;
			}
		}

		return -1;
	}

	/*
	 * Calculates the distance from a specified point to a point on the path
	 */
	private double calcDistance(double x, double y, Segment seg) {
		double dx = seg.x - x;
		double dy = seg.y - y;

		return Math.sqrt(dx * dx + dy * dy);
	}


	private class SpeedContainer {
		public double leftSpeed, rightSpeed;

		public SpeedContainer(double leftSpeed, double rightSpeed) {
			this.leftSpeed = leftSpeed;
			this.rightSpeed = rightSpeed;
		}
	}
}
