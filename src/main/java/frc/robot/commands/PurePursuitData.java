// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

/** Add your docs here. */
public class PurePursuitData {

    public double k_maxSpeed = 5.000000;
    public double k_maxAccel = 5.000000;
    public double k_maxDecl = 5.000000;
    public double k_maxJerk = 100;

    public PurePursuitData() {

    }

    public PurePursuitData(double maxSpeed, double maxAccel, double maxDecl, double maxJerk) {
        k_maxSpeed = maxSpeed;
        k_maxAccel = maxAccel;
        k_maxDecl = maxDecl;
        k_maxJerk = maxJerk;    
    }

    PurePursuitData(double maxSpeed){
        k_maxSpeed = maxSpeed; 
    }
}