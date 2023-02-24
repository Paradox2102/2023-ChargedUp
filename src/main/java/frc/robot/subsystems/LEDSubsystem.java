// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robotCore.LED5050String;
import frc.robotCore.LED5050String.RGBColor;

public class LEDSubsystem extends SubsystemBase {
  private LED5050String m_LED;
  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {}

  public void setColor(RGBColor[] colors) {
    m_LED.SetColors(colors);
    m_LED.Update();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
