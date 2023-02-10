// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.ApriltagsCamera.ApriltagsCamera;
import frc.ApriltagsCamera.Logger;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.Autos;
import frc.robot.commands.CalibrateDrive;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualArmCommand;
import frc.robot.commands.ManualReachCommand;
import frc.robot.commands.PathFollowingCommand;
import frc.robot.commands.SetBrakeCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ReachSubsystem;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Variables that will be passed into commands and subsystems
  public AprilTagFieldLayout m_tags;
  public final ApriltagsCamera m_camera = new ApriltagsCamera();

  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public final DriveSubsystem m_driveSubsystem;
  private final ReachSubsystem m_reachSubsystem = new ReachSubsystem();
  private final ArmSubsystem m_armSubsystem = new ArmSubsystem(m_reachSubsystem);
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();

  // Driver 1 Controller
  private final CommandXboxController m_xbox1;// = new CommandXboxController(0);
  private final CommandJoystick m_joystick1;

  // Driver 2 Controller
  private final CommandJoystick m_stick2 = new CommandJoystick(1);


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    try {
      // change this later
      AprilTagFieldLayout tags = new AprilTagFieldLayout("/home/lvuser/deploy/2023-chargedup.json");
      Logger.log("RobotContainer", 1, tags.toString());
      Logger.log("RobotContainer", 1, tags.getTagPose(1).get().toString());
      Logger.log("RobotContainer", 1, String.format("%f", tags.getTagPose(1).get().getRotation().getZ()));
      m_tags = tags;
    } catch (IOException e) {
      Logger.log("RobotContainer", 1, "Field didn't load");
    }
    m_camera.connect("10.21.2.10", 5800);

    m_driveSubsystem = new DriveSubsystem(m_camera, m_tags);

    // Choose which Joystick Driver 1 wants
    if (Constants.k_xboxController) {
      m_xbox1 = new CommandXboxController(0);
      m_joystick1 = null;
    } else {
      m_joystick1 = new CommandJoystick(0);
      m_xbox1 = null;
    }

    // Configure the trigger bindings
    configureBindings();
  }

  public void initialize() {
    m_armSubsystem.resetAngles();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.

    // Driver 1
    if (Constants.k_xboxController) {
      Trigger directionSwitch = m_xbox1.rightBumper();
      m_driveSubsystem.setDefaultCommand(new ArcadeDriveCommand(m_driveSubsystem, () -> m_xbox1.getLeftX(), () -> m_xbox1.getRightY(), new ToggleTrigger(directionSwitch.debounce(.1))));
      m_xbox1.b().toggleOnTrue(new PathFollowingCommand(m_driveSubsystem, () -> m_xbox1.getRightY()));
    } else {
      m_driveSubsystem.setDefaultCommand(new ArcadeDriveCommand(m_driveSubsystem, () -> m_joystick1.getX(), () -> m_joystick1.getY(), () -> m_joystick1.getThrottle() < 0));
      m_joystick1.button(1).onTrue(new PathFollowingCommand(m_driveSubsystem, null));// () -> -m_joystick1.getY()));

      m_joystick1.button(2).toggleOnTrue(new PathFollowingCommand(m_driveSubsystem, null));
      m_joystick1.button(3).whileTrue(new CalibrateDrive(m_driveSubsystem));
      m_joystick1.button(4).toggleOnTrue(new SetBrakeCommand(m_armSubsystem));
    }

    // Driver 2
    m_stick2.button(8).whileTrue(new ManualReachCommand(m_reachSubsystem, .5));
    m_stick2.button(7).whileTrue(new ManualReachCommand(m_reachSubsystem, -.5));
    m_stick2.button(2).whileTrue(new IntakeCommand(m_intakeSubsystem, -0.8));
    m_stick2.button(1).whileTrue(new IntakeCommand(m_intakeSubsystem, 1));
    m_armSubsystem.setDefaultCommand(new ManualArmCommand(m_armSubsystem, () -> m_stick2.getY()));
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
