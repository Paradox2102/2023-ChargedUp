// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

// Intake Subsystem 
public class IntakeSubsystem extends SubsystemBase {

  // Left and right TalonSRX motors for the IntakeSubsystem 
  private TalonSRX m_leftIntakeMotor, m_rightIntakeMotor;
  private Solenoid m_brake = new Solenoid(PneumaticsModuleType.REVPH, Constants.k_claw);
  private PowerDistribution m_powerDistribution = new PowerDistribution();
  public enum ClawPosition {OPEN, CUBE, CONE}
  private CANSparkMax m_clawMotor = new CANSparkMax(Constants.k_clawMotor, MotorType.kBrushless);
  boolean m_commandEnded = false;

  RelativeEncoder m_clawEncoder = m_clawMotor.getEncoder();
  private final double k_clawTicksToDegrees = 90/6.928; //173.08;

  private final double k_clawStartingAngle = 0;
  private final double k_clawDeadZoneInDegrees = 2;

  private double m_power = 0;
  private boolean m_isAuto = false;

  private double m_clawTargetAngleInDegrees = k_clawStartingAngle;

  // Claw PID
  private final double k_clawP = 0.02;
  private final double k_clawI = 0;
  private final double k_clawD = 0;
  PIDController m_clawPID = new PIDController(k_clawP, k_clawI, k_clawD);
  private final double k_clawF = 0;
  private boolean m_enabled = false; 
  
  // Digital Input Limit switch for the IntakeSubsystem 
  
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    // Initialize motors 
    m_leftIntakeMotor = new TalonSRX(Constants.k_leftIntakeMotor);
    m_rightIntakeMotor = new TalonSRX(Constants.k_rightIntakeMotor);
    m_leftIntakeMotor.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute); 
    // Set proper inversions
    m_leftIntakeMotor.setInverted(false);
    m_rightIntakeMotor.setInverted(true);
    m_leftIntakeMotor.setNeutralMode(NeutralMode.Brake);
    m_rightIntakeMotor.setNeutralMode(NeutralMode.Brake);
    m_clawMotor.restoreFactoryDefaults();
    m_clawMotor.setInverted(true);
    resetClawPosition();
    m_clawMotor.setIdleMode(IdleMode.kBrake);

    m_clawEncoder.setPositionConversionFactor(k_clawTicksToDegrees);
  }

  // Sets power of left and right motors of IntakeSubsystem
  public void setPower(double power) {
    m_leftIntakeMotor.set(ControlMode.PercentOutput, power);
    m_rightIntakeMotor.set(ControlMode.PercentOutput, -power);
  }

  public boolean isClawOnTarget() {
    return Math.abs(getClawAngleInDegrees() - getClawAngleInDegrees()) <= k_clawDeadZoneInDegrees;
  }

  public void resetClawPosition() {
    m_clawEncoder.setPosition(0);
  }

  public void setClaw(ClawPosition position) {
    if(!Constants.k_isCompetition){
      switch (position) {
        case OPEN: 
        case CUBE:
          m_brake.set(false);
          break;
        case CONE:
          m_brake.set(true);
          break;
      }
    } else {
      switch(position) {
        case OPEN:
          m_enabled = true; 
          m_clawTargetAngleInDegrees = 0;
          break;
        case CUBE:
          m_enabled = true; 
          m_clawTargetAngleInDegrees = 77;
          break;
        case CONE:
          m_enabled = false; 
          setClawPower(0.2); 
          // m_clawTargetAngleInDegrees = 90;
          break;
      }
    }
  }

  public void setClawPower(double clawPower) {
    m_clawMotor.set(clawPower);
  }

  public void setClawAngle(double degrees) {
    // m_clawEncoder.setPosition(degrees);
  }

  public void stop() {
    setPower(0);
  }

  public double getClawAngleInDegrees() {
    return m_clawEncoder.getPosition();
  }

  public double getWristFeedforward() {
    return k_clawF;
  }

  public double getMagEncoderPosition() {
    if (Constants.k_isCompetition) {
      return m_rightIntakeMotor.getSelectedSensorPosition(); 
    } else {
      return m_leftIntakeMotor.getSelectedSensorPosition(); 
    }
  }

  public double getMagEncoderVelocity() {
    if (Constants.k_isCompetition) {
      return m_rightIntakeMotor.getSelectedSensorVelocity(); 
    } else {
      return m_leftIntakeMotor.getSelectedSensorVelocity(); 
    }
  }

  public void setPowerAutoPeriod(double power) {
    m_power = power;
    m_isAuto = true;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Left Intake Current", m_powerDistribution.getCurrent(Constants.k_leftIntakeMotor));
    // SmartDashboard.putNumber("Right Intake Current", m_powerDistribution.getCurrent(Constants.k_rightIntakeMotor));
    // SmartDashboard.putNumber("Claw Angle in Degrees", getClawAngleInDegrees());

    if (m_enabled) {
      double clawPower = m_clawPID.calculate(getClawAngleInDegrees(), m_clawTargetAngleInDegrees);
      if (Constants.k_isCompetition) {
        m_clawMotor.set(clawPower);
      }
      SmartDashboard.putNumber("Claw Power", clawPower);
    } else {
      setClawPower(.2);
    }

    if (m_isAuto) {
      setPower(m_power);
    }
  
    // System.out.println(String.format("Arm Power = %f", armPower));
    // }
    // if (isClawOnTarget()) {
    // m_claw.set(0);
    // } else {
    // double clawPower = m_clawPID.calculate(getClawAngle(),
    // m_clawTargetAngleInDegrees);
    // m_claw.set(clawPower + getClawAngle());
  }

}
