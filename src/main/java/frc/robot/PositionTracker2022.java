package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.ApriltagsCamera.ApriltagsCamera;

public class PositionTracker2022 extends PositionTracker {
	private double m_x;
	private double m_y;
	private SensorData m_sensors;
		
	private double m_prevLeft;
	private double m_prevRight;	
	private double m_lastAngle;
	
	private boolean m_runPositionThread = true;
	
	private Object lock = new Object();
	
	private PositionThread m_thread = new PositionThread();

	public PositionTracker2022(double x, double y, SensorData sensor) {
		super();
		
		m_sensors = sensor;
				
		m_prevLeft = getLeftEncoderPos();
		m_prevRight = getRightEncoderPos();
		m_lastAngle = getGyroAngleDegrees();
		
		// m_thread.start();
	}

	/*
	 * Returns the dx and dy offsets to the new position
	 */
	double[] computeNewPosition(double leftPos, double rightPos, double curAngle, double prevLeft, double prevRight, double lastAngle)
	{
		double arcDist = ((leftPos - prevLeft) + (rightPos - prevRight))/2.0;
		
		double meanAngle = (curAngle + lastAngle)/2.0;
		double halfAngle =(curAngle - lastAngle)/2.0;

		meanAngle = Math.toRadians(meanAngle);
		halfAngle = Math.toRadians(halfAngle);

		double dist = arcDist * (1.0- (halfAngle * halfAngle)/6.0);
		
		return new double[] { dist * Math.cos(meanAngle), dist * Math.sin(meanAngle) };
	}

	private void updatePos() {
		double leftPos = getLeftEncoderPos();
		double rightPos = getRightEncoderPos();
		double curAngle = getGyroAngleDegrees();

		m_posServer.setAllianceColor(DriverStation.getAlliance() == DriverStation.Alliance.Red);

		synchronized(lock) {
			double[] offset = computeNewPosition(leftPos, rightPos, curAngle, m_prevLeft, m_prevRight, m_lastAngle); 

			// double arcDist = ((leftPos - m_prevLeft) + (rightPos - m_prevRight))/2.0;
			
			// double meanAngle = (curAngle + m_lastAngle)/2.0;
			// double halfAngle =(curAngle - m_lastAngle)/2.0;

			// meanAngle = Math.toRadians(meanAngle);
			// halfAngle = Math.toRadians(halfAngle);

			// double dist = arcDist * (1.0- (halfAngle * halfAngle)/6.0);
			
			m_x += offset[0];
			m_y += offset[1];
			
			m_prevLeft = leftPos;
			m_prevRight = rightPos;
			m_lastAngle = curAngle;
		}
	}
	
	public PositionContainer getPos() {
		synchronized (lock) {
			return new PositionContainer(m_x, m_y);
		}
	}

	@Override
	public Pose2d getPose2d() {
		synchronized (lock) {
			return new Pose2d(m_x, m_y, Rotation2d.fromDegrees(m_lastAngle));
		}
	}

	@Override
	public void setXYAngle(double x, double y, double angle)
	{
		synchronized (lock) {
			m_x = x;
			m_y = y;
			m_sensors.getGyro().setYaw(angle);
			m_lastAngle = angle;
		}		
	}

	@Override
	public double getLeftEncoderPos() {
		return m_sensors.getLeftEncoderPos();
	}
	
	@Override
	public double getRightEncoderPos() {
		return m_sensors.getRightEncoderPos();
	}

	@Override
	public double getLeftEncoderVel() {
		return m_sensors.getLeftEncoderVel();
	}

	@Override
	public double getRightEncoderVel() {
		return m_sensors.getRightEncoderVel();
	}
	
	public double getGyroAngleDegrees() {
		return m_sensors.getAngle();
	}
	
	public class PositionThread extends Thread{
		public void run() {
			long step = 20;
			for(long nextRun = System.currentTimeMillis();;nextRun += step) {
				if(m_runPositionThread) {
					updatePos();
				}
				try {
					long sleepTime = nextRun - System.currentTimeMillis();
					if(sleepTime > 0) {
						sleep(sleepTime);
					}
				} catch (InterruptedException e) {
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
	}
	
	public void startPosUpdate() {
		m_runPositionThread = true;
	}
	
	public void stopPosUpdate() {
		m_runPositionThread = false;
	}

	@Override
	public void update(ApriltagsCamera frontCamera, ApriltagsCamera rearCamera) {
		updatePos();

		Pose2d pos = getPose2d();
		
		m_posServer.setPosition(pos.getX(), pos.getY(), pos.getRotation().getDegrees());
	}	
}