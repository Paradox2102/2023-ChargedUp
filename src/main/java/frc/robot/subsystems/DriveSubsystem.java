// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ApriltagsCamera.ApriltagsCamera;
import frc.ApriltagsCamera.ApriltagsCamera.ApriltagsCameraRegion;
import frc.ApriltagsCamera.ApriltagsCamera.ApriltagsCameraRegions;
import frc.pathfinder.Pathfinder.Path;
import frc.robot.Constants;
import frc.robot.LocationTracker;
import frc.robot.Navigator;
import frc.robot.PositionTracker;
import frc.robot.PurePursuit;
import frc.robot.RobotContainer;
import frc.robot.Sensor;

public class DriveSubsystem extends SubsystemBase {
  ApriltagsCamera m_camera;
  WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(0);
  LocationTracker m_tracker = new LocationTracker();
  private final Field2d m_field = new Field2d();
  AprilTagFieldLayout m_tags;
  public PurePursuit m_pursuitFollower;

  private final double k_maxSpeed = 19000; 
  private final double k_p = 0.1;
  private final double k_i = 0.002; 
  private final double k_f = 0.051; 
  private final double k_iZone = 300; 
  private final int k_timeout = 30; 
  private final double k_rampTimeSeconds = .25;

  Object m_setlock = new Object();

  // motors
  TalonFX m_rightDrive = new TalonFX(Constants.k_rightDrive);
  TalonFX m_rightFollower = new TalonFX(Constants.k_rightFollower);
  TalonFX m_leftDrive = new TalonFX(Constants.k_leftDrive);
  TalonFX m_leftFollower = new TalonFX(Constants.k_leftFollower);

  Navigator m_navigator;
  private Sensor m_sensors;
  private PositionTracker m_posTracker;

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem(ApriltagsCamera camera, AprilTagFieldLayout tags) {
    SmartDashboard.putData("Field", m_field);
    m_camera = camera;
    m_tags = tags;
    m_gyro.reset();
    SmartDashboard.putNumber("True X", 0);
    SmartDashboard.putNumber("True Y", 0);
    SmartDashboard.putNumber("True Yaw", 0);


    m_leftFollower.follow(m_leftDrive);
    m_rightFollower.follow(m_rightDrive);

    m_rightDrive.setInverted(false);
    m_rightFollower.setInverted(false);

    m_leftDrive.setInverted(true);
    m_leftFollower.setInverted(true);

    m_leftDrive.config_kF(0, k_f, k_timeout); 
    m_leftDrive.config_kP(0, k_p, k_timeout);
    m_leftDrive.config_kI(0, k_i, k_timeout);
    m_leftDrive.config_IntegralZone(0, k_iZone, k_timeout);
    m_rightDrive.config_kF(0, k_f, k_timeout); 
    m_rightDrive.config_kP(0, k_p, k_timeout);
    m_rightDrive.config_kI(0, k_i, k_timeout);
    m_rightDrive.config_IntegralZone(0, k_iZone, k_timeout);

    m_sensors = new Sensor(() -> m_leftDrive.getSelectedSensorPosition(), () -> m_rightDrive.getSelectedSensorPosition(), () -> m_leftDrive.getSelectedSensorVelocity(), () -> m_rightDrive.getSelectedSensorVelocity(), m_gyro);
    m_posTracker = new PositionTracker(0, 0, false, m_sensors);
    m_navigator = new Navigator(m_posTracker);
    m_navigator.reset(90, 0, 0);
    m_pursuitFollower = new PurePursuit(m_navigator, (l, r) -> setSpeedFPS(l, r), 20);
    m_pursuitFollower.enableLogging("/home/lvuser/logs");
  
  }

  public void setSpeed(double leftSpeed, double rightSpeed) {
    synchronized(m_setlock){
      m_rightDrive.set(TalonFXControlMode.Velocity, rightSpeed * k_maxSpeed);
      m_leftDrive.set(TalonFXControlMode.Velocity, leftSpeed * k_maxSpeed);
    }
  }

  public void setSpeedFPS(double leftSpeed, double rightSpeed)
  {
    // // Change speed from FPS to -1 to 1 range
    leftSpeed = leftSpeed * 1.0/10 * 1.0/Constants.k_feetPerTick;
    rightSpeed =  rightSpeed * 1.0/10 * 1.0/Constants.k_feetPerTick;

    // Logger.Log("DriveSubsystem", 1, String.format("ls=%f,rs=%s", leftSpeed, rightSpeed));

    // Logger.Log("DriveSubsystem", 1, String.format("setSpeedFPS: left=%f, right=%f", leftSpeed, rightSpeed));

    setSpeed(leftSpeed, rightSpeed);
  }

  public void setPower(double leftPower, double rightPower) {
    synchronized(m_setlock){
      m_rightDrive.set(ControlMode.PercentOutput, rightPower);
      m_leftDrive.set(ControlMode.PercentOutput, leftPower);
      }
  }

  public void startPath(Path path, boolean isReversed, boolean setPosition, DoubleSupplier speed) {
    m_pursuitFollower.loadPath(path, isReversed, true, setPosition, speed);
    m_pursuitFollower.startPath();
  }

  public void endPath() {
    m_pursuitFollower.stopFollow();
  }

  public boolean isPathFinished() {
    return (m_pursuitFollower.isFinished());
  }

  public void stop() {
    setPower(0, 0);
  }

  public void setBrakeMode(boolean brake){
    NeutralMode mode = brake ? NeutralMode.Brake : NeutralMode.Coast;
    m_leftDrive.setNeutralMode(mode);
    m_leftFollower.setNeutralMode(mode);
    m_rightDrive.setNeutralMode(mode);
    m_rightFollower.setNeutralMode(mode);
  }

  public void resetGyro(double angle) {
    m_gyro.reset();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("Navigator X", m_navigator.getPos().x);
    SmartDashboard.putNumber("Navigator Y", m_navigator.getPos().y);
    SmartDashboard.putNumber("Left Position", m_leftDrive.getSelectedSensorPosition());

    // This method will be called once per scheduler run
    ApriltagsCameraRegions regions = m_camera.getRegions();
    if (regions != null && regions.getRegionCount() >= 1) {
      SmartDashboard.putNumber("Region Count", regions.getRegionCount());
      ApriltagsCameraRegion region = regions.getRegion(0);
      double[] tag = m_tracker.getTag(region.m_tag, m_tags);
      if (tag != null) {
        Pose2d location = m_tracker.getLocation(region.m_tvec[0], region.m_tvec[2], m_gyro.getAngle(), tag);
        m_field.setRobotPose(location);
        SmartDashboard.putNumberArray("T vector", region.m_tvec);
        SmartDashboard.putNumberArray("R vector", region.m_rvec);
        SmartDashboard.putNumber("Gyro Angle", m_gyro.getAngle());
      }      
      SmartDashboard.putNumber("Tag ID", region.m_tag);
      SmartDashboard.putBoolean("Has Regions", true);
    } else {
      SmartDashboard.putBoolean("Has Regions", false);
    }
  }
}
