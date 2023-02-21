package frc.robot;

import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import frc.ApriltagsCamera.Logger;

// import frc.lib.CSVWriter;
// import frc.lib.CSVWriter.Field;

public class PositionTracker implements Tracker {
	private SensorData m_sensors;
	private DifferentialDrivePoseEstimator m_poseEstimator;

	public PositionTracker(double x, double y, SensorData sensor) {
		m_sensors = sensor;
		m_poseEstimator = new DifferentialDrivePoseEstimator(new DifferentialDriveKinematics(Constants.k_wheelBase),
		ParadoxField.rotation2dFromParadoxAngle(Constants.k_startAngleDegrees), 
		0, 0, ParadoxField.pose2dFromParadox(0, 0, Constants.k_startAngleDegrees));
	}
	
	
	public PositionContainer getPos() {
		Pose2d pose = ParadoxField.pose2dFromFRC(m_poseEstimator.getEstimatedPosition());
		return new PositionContainer(pose.getX(), pose.getY());
	}

	public void setXYAngle(double x, double y, double angleInDegrees) {
		Logger.log("PositionTracker", 1, String.format("x=%f, y=%f, angle=%f", x, y, angleInDegrees));
		m_sensors.getGyro().setYaw(-angleInDegrees);
		m_poseEstimator.resetPosition(
			ParadoxField.rotation2dFromParadoxAngle(angleInDegrees), 
			ParadoxField.distanceFromParadox(m_sensors.getLeftEncoderPos()), 
			ParadoxField.distanceFromParadox(m_sensors.getRightEncoderPos()), 
			ParadoxField.pose2dFromParadox(x, y, angleInDegrees)
		);
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
		m_poseEstimator.update(ParadoxField.rotation2dFromParadoxAngle(getAngle()), 
		ParadoxField.distanceFromParadox(getLeftEncoderPos()), 
		ParadoxField.distanceFromParadox(getRightEncoderPos()));
	} 
}