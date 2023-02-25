// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ApriltagsCamera.ApriltagsCamera;
import frc.pathfinder.Pathfinder.Path;
import frc.robot.Constants;
import frc.robot.LocationTracker;
import frc.robot.Navigator;
import frc.robot.ParadoxField;
import frc.robot.PositionTracker;
import frc.robot.PurePursuit;
import frc.robot.Sensor;

public class DriveSubsystem extends SubsystemBase {
  ApriltagsCamera m_frontCamera;
  ApriltagsCamera m_backCamera; 
  WPI_PigeonIMU m_gyro = new WPI_PigeonIMU(0);
  LocationTracker m_tracker = new LocationTracker();
  private final Field2d m_field = new Field2d();
  AprilTagFieldLayout m_tags;
  public PurePursuit m_pursuitFollower;

  // private final double k_maxSpeed = 20000; 
  // private final double k_rampTimeSeconds = .25;

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
  public DriveSubsystem(ApriltagsCamera frontCamera, ApriltagsCamera backCamera, AprilTagFieldLayout tags) {
    SmartDashboard.putData("Field", m_field);
    m_frontCamera = frontCamera;
    m_backCamera = backCamera; 
    m_tags = tags;
    m_gyro.reset();
    SmartDashboard.putNumber("True X", 0);
    SmartDashboard.putNumber("True Y", 0);
    SmartDashboard.putNumber("True Yaw", 0);

    m_rightDrive.setSelectedSensorPosition(0);
    m_leftDrive.setSelectedSensorPosition(0);

    m_leftFollower.follow(m_leftDrive);
    m_rightFollower.follow(m_rightDrive);

    m_rightDrive.setInverted(false);
    m_rightFollower.setInverted(false);

    m_leftDrive.setInverted(true);
    m_leftFollower.setInverted(true);

    // Set PID values
    m_leftDrive.config_kF(0, Constants.k_driveF, Constants.k_timeout); 
    m_leftDrive.config_kP(0, Constants.k_driveP, Constants.k_timeout);
    m_leftDrive.config_kI(0, Constants.k_driveI, Constants.k_timeout);
    m_leftDrive.config_IntegralZone(0, Constants.k_DriveIZone, Constants.k_timeout);
    m_rightDrive.config_kF(0, Constants.k_driveF, Constants.k_timeout); 
    m_rightDrive.config_kP(0, Constants.k_driveP, Constants.k_timeout);
    m_rightDrive.config_kI(0, Constants.k_driveI, Constants.k_timeout);
    m_rightDrive.config_IntegralZone(0, Constants.k_DriveIZone, Constants.k_timeout);

    m_sensors = new Sensor(() -> m_leftDrive.getSelectedSensorPosition(), () -> m_rightDrive.getSelectedSensorPosition(), () -> m_leftDrive.getSelectedSensorVelocity(), () -> m_rightDrive.getSelectedSensorVelocity() , m_gyro);
    m_posTracker = new PositionTracker(0, 0, m_sensors);
    m_navigator = new Navigator(m_posTracker);
    m_navigator.reset(0, 0, 0);
    m_pursuitFollower = new PurePursuit(m_navigator, (l, r) -> setSpeedFPS(l, r), 20);
    m_pursuitFollower.enableLogging("/home/lvuser/logs");
  
  }

  public void setSpeed(double leftSpeed, double rightSpeed) {
    // Takes speed in ticks
    synchronized(m_setlock){
      m_rightDrive.set(TalonFXControlMode.Velocity, rightSpeed);
      m_leftDrive.set(TalonFXControlMode.Velocity, leftSpeed);
      SmartDashboard.putNumber("Right Speed", rightSpeed);
		  SmartDashboard.putNumber("Left Speed", leftSpeed);
    }
  }

  public PositionTracker getTracker() {
    return m_posTracker;
  }

  public void setSpeedFPS(double leftSpeed, double rightSpeed)
  {
    // Change speed from FPS to -1 to 1 range
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

  public void resetEncoders() {
    m_leftDrive.setSelectedSensorPosition(0);
    m_rightDrive.setSelectedSensorPosition(0);
  }

  // Start autonomous path during Teleop
  public void startPath(Path path, boolean isReversed, boolean setPosition, DoubleSupplier speed) {
    m_pursuitFollower.loadPath(path, isReversed, true, setPosition, speed);
    m_pursuitFollower.startPath();
  }

  public void endPath() {
    m_pursuitFollower.stopFollow();
  }

  public double findTargetAngle(double x0, double y0) {
    double y = m_posTracker.getPose2d().getY() - y0;
    double x = x0 - m_posTracker.getPose2d().getX();
    double m_targetAngle = -Math.atan2(y, x);
    SmartDashboard.putNumber("Target Angle", Math.toDegrees(m_targetAngle));
    return m_targetAngle;
  }

  public boolean isPathFinished() {
    return (m_pursuitFollower.isFinished());
  }

  public void stop() {
    setPower(0, 0);
  }

  public double getLeftPos() {
    return m_leftDrive.getSelectedSensorPosition() * Constants.k_feetPerTick;
  }

  public double getRightPos() {
    return m_rightDrive.getSelectedSensorPosition() * Constants.k_feetPerTick;
  }

  public double getPitch() {
    if (Constants.k_isCompetition) {
      return m_gyro.getPitch();
    } else {
      return m_gyro.getRoll();
    }
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

  public Sensor getSensors(){
    return m_sensors;
  } 

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Left Pos", m_leftDrive.getSelectedSensorPosition() * Constants.k_feetPerTick);
    SmartDashboard.putNumber("Right Pos", m_rightDrive.getSelectedSensorPosition() * Constants.k_feetPerTick);
    SmartDashboard.putNumber("Navigator X", m_navigator.getPos().x);
    SmartDashboard.putNumber("Navigator Y", m_navigator.getPos().y);
    SmartDashboard.putNumber("Left Position", m_leftDrive.getSelectedSensorPosition());
    SmartDashboard.putNumber("Left Vel (feet)", m_leftDrive.getSelectedSensorVelocity() * 10 * Constants.k_feetPerTick);
    SmartDashboard.putNumber("Right Vel (feet)", m_rightDrive.getSelectedSensorVelocity() * 10 * Constants.k_feetPerTick);
    SmartDashboard.putNumber("Gyro Yaw", m_gyro.getAngle());
    SmartDashboard.putNumber("Gyro Roll", m_gyro.getRoll());
    SmartDashboard.putNumber("Navigator Angle", ParadoxField.normalizeAngle(m_posTracker.getPose2d().getRotation().getDegrees()));

    m_field.setRobotPose(m_navigator.getPose2d());

    m_posTracker.update(m_frontCamera);
 
    // m_backCamera.processRegions(m_poseEstimator, tags); 
  }
}
