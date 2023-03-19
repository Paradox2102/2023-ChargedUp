package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

/*
 * This class provides the base class for the old and new intake subsystems
 * Using this class will allow us to interchange the two with minimal
 * changes to the rest of the code.
 */
public abstract class IntakeSubsystem  extends SubsystemBase {
    public enum ClawPosition {OPEN, CUBE, CONE}

    abstract public void setClaw(ClawPosition position);
    abstract public double getMagEncoderPosition();
    abstract public void setPower(double power);
    abstract public void stop();
    abstract public void setPowerAutoPeriod(double power);
    abstract public void setClawPower(double clawPower);
    abstract public double getMagEncoderVelocity();
}
