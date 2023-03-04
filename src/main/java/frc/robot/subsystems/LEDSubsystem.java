// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class LEDSubsystem extends SubsystemBase {

  AddressableLED m_leftLED = new AddressableLED(Constants.k_leftLED); // blue side
  // AddressableLED m_rightLED = new AddressableLED(Constants.k_rightLED); // red side
  AddressableLEDBuffer m_leftBuffer = new AddressableLEDBuffer(Constants.k_leftLength);
  AddressableLEDBuffer m_rightBuffer = new AddressableLEDBuffer(Constants.k_rightLength);

  // Red RGB (255, 0, 0)
  // Blue RGB (0, 0, 255)
  // Yellow RGB (252, 215, 3)
  // Purple RGB (209, 0, 204)

  /** Creates a new LEDSubsystem. */
  public LEDSubsystem() {
    m_leftLED.setLength(Constants.k_leftLength);
    // m_rightLED.setLength(Constants.k_rightLength);

    m_leftLED.setData(m_leftBuffer);
    // m_rightLED.setData(m_rightBuffer);
    m_leftLED.start();
    // m_rightLED.start();
  }

  // Paradox Colors
  public void setStateIdle() {
    for (int i = 0; i < Constants.k_leftLength; i++) {
      if (i % 2 == 0) {
        if (i <= Constants.k_leftLength / 3) {
          m_leftBuffer.setRGB(i, 255, 0, 0);
        } else if (i <= 2 * Constants.k_leftLength / 3) {
          m_leftBuffer.setRGB(i, 252, 215, 3);
        } else {
          m_leftBuffer.setRGB(i, 0, 0, 255);
        }
      }
    }
    for (int j = 0; j < Constants.k_rightLength; j++) {
      if (j % 2 == 0) {
        if (j <= Constants.k_leftLength / 3) {
          m_leftBuffer.setRGB(j, 255, 0, 0);
        } else if (j <= 2 * Constants.k_rightLength / 3) {
          m_leftBuffer.setRGB(j, 252, 215, 3);
        } else {
          m_leftBuffer.setRGB(j, 0, 0, 255);
        }
      }
    }
    m_leftLED.setData(m_leftBuffer);
    // m_rightLED.setData(m_rightBuffer);
  }

  // Yellow
  public void setStateCone() {
    for (int i = 0; i < Constants.k_leftLength; i++) {
      if (i % 2 == 0) {
        m_leftBuffer.setRGB(i, 252, 215, 3);
      }
    }
    for (int j = 0; j < Constants.k_rightLength; j++) {
      if (j % 2 == 0) {
        m_rightBuffer.setRGB(j, 252, 215, 3);
      }
    }
    m_leftLED.setData(m_leftBuffer);
    // m_rightLED.setData(m_rightBuffer);
  }

  // Purple
  public void setStateCube() {
    for (int i = 0; i < Constants.k_leftLength; i++) {
      if (i % 2 == 0) {
        m_leftBuffer.setRGB(i, 209, 0, 224);
      }
    }
    for (int j = 0; j < Constants.k_rightLength; j++) {
      if (j % 2 == 0) {
        m_rightBuffer.setRGB(j, 209, 0, 224);
      }
    }
    m_leftLED.setData(m_leftBuffer);
    // m_rightLED.setData(m_rightBuffer);
  }

  // Red
  public void setRed() {
    for (int i = 0; i < Constants.k_leftLength; i++) {
      m_leftBuffer.setRGB(i, 255, 0, 0);
      System.out.println("Setting red");
    }
    m_leftLED.setData(m_leftBuffer);
    System.out.println("Set data");
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
