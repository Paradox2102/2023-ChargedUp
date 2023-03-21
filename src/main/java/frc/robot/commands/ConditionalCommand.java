// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConditionalCommand extends SequentialCommandGroup {
  /** Creates a new ConditionalCommand. */
  // PROBLEM: This won't work because the condition is evaluated at construction time.  It needs to be in the initialize method.  Why create this at all?  Why not just use the one WPILIB provides? -Gavin
  public ConditionalCommand(Command command1, Command command2, BooleanSupplier runCommand1) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      runCommand1.getAsBoolean() ? command1 : command2
    );
  }
}
