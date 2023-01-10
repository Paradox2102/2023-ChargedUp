package frc.robot;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class ToggleTrigger implements BooleanSupplier {
    private boolean m_state = true;

    public ToggleTrigger (Trigger trigger) {
        trigger.toggleOnTrue(new InstantCommand(() -> m_state = !m_state));
    }

    public boolean getAsBoolean() {
        return m_state;
    }
}
