package frc.robot;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

// import frc.lib.CSVWriter;
// import frc.lib.CSVWriter.Field;

public class PositionTracker implements Tracker {
	private SensorData m_sensors;
	private DifferentialDrivePoseEstimator m_poseEstimator;

	public PositionTracker(double x, double y, SensorData sensor) {
		m_sensors = sensor;
		m_poseEstimator = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(Constants.k_wheelBase),
		ParadoxField.Rotation2dFromParadoxAngle(Constants.k_startAngleDegrees), 
		0, 0, ParadoxField.Pos2dFromParadox(0, 0, Constants.k_startAngleDegrees));
	}
	
	
	public PositionContainer getPos() {
		Pose2d pose = ParadoxField.pos2dFromFRC(m_poseEstimator.getEstimatedPosition());
		return new PositionContainer(pose.getX(), pose.getY());
	}

	public void setXY(double x, double y) {
		
	}

	public void setAngle(double angle) {
		m_sensors.getGyro().setYaw(angle);
	}

	public double getLeftEncoderPos() {
		return m_sensors.getLeftEncoderPos();
	}

	public double getRightEncoderPos() {
		return m_sensors.getRightEncoderPos();
	}

	public double getLeftEncoderVel() {
		return m_sensors.getLeftEncoderVel();
	}

	public double getRightEncoderVel() {
		return m_sensors.getRightEncoderVel();
	}

	public double getAngle() {
		return m_sensors.getAngle();
	}

	public static class PositionContainer {
		public double x, y;

		public PositionContainer(double x, double y) {
			this.x = x;
			this.y = y;
		}
	}
	public void update(){
		m_poseEstimator.update(ParadoxField.Rotation2dFromParadoxAngle(getAngle()), 
		ParadoxField.distanceFromParadox(getLeftEncoderPos()), 
		ParadoxField.distanceFromParadox(getRightEncoderPos()));
	} 
}