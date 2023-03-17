// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.ApriltagsCamera.ApriltagsCamera;
import frc.ApriltagsCamera.Logger;
import frc.robot.commands.ArcadeDriveCommand;
import frc.robot.commands.BrakeOffCommand;
import frc.robot.commands.DeliverGamePieceCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ManualAdjustArmAngle;
import frc.robot.commands.ManualArmCommand;
import frc.robot.commands.ManualClawCommand;
import frc.robot.commands.ManualReachCommand;
import frc.robot.commands.PathFollowingCommand;
import frc.robot.commands.SetArmPositionExtent;
import frc.robot.commands.SetArmZeroCommand;
import frc.robot.commands.SetClawCommand;
import frc.robot.commands.SetLEDCommand;
import frc.robot.commands.Autos.Auto_1LA2LB3L;
import frc.robot.commands.Autos.Auto_1LA2MB2H;
import frc.robot.commands.Autos.Auto_1MA2M;
import frc.robot.commands.Autos.Auto_2LA;
import frc.robot.commands.Autos.Auto_2LA2M;
import frc.robot.commands.Autos.Auto_4LBS;
import frc.robot.commands.Autos.Auto_4LS;
import frc.robot.commands.Autos.Auto_4MS;
import frc.robot.commands.Autos.Auto_5L;
import frc.robot.commands.Autos.Auto_9LD;
import frc.robot.commands.Autos.Auto_9MD;
import frc.robot.commands.Autos.Auto_9LD9M;
import frc.robot.commands.Autos.Auto_9HD;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.ReachSubsystem;

import java.io.IOException;
import java.util.function.BooleanSupplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  @SuppressWarnings("unused")
  private final Constants m_constants = new Constants();
  
  // Variables that will be passed into commands and subsystems
  public AprilTagFieldLayout m_tags;
  public final ApriltagsCamera m_frontCamera = new ApriltagsCamera(Constants.k_xFrontCameraOffsetInches, 0, Constants.k_frontCameraAngle);
  public final ApriltagsCamera m_backCamera = new ApriltagsCamera(Constants.k_xRearCameraOffsetInches, 0, Constants.k_rearCameraAngle);

  // The robot's subsystems and commands are defined here...

  private final LEDSubsystem m_LEDSubsystem = new LEDSubsystem();
  public final DriveSubsystem m_driveSubsystem;
  private final ReachSubsystem m_reachSubsystem = new ReachSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  //Intake Subsystem only needed to get absolute mag encoder for arm 
  private final ArmSubsystem m_armSubsystem;
  // Driver 1 Controller
  private final CommandXboxController m_xbox1;// = new CommandXboxController(0);
  private final CommandJoystick m_joystick1;

  // Driver 2 Controller
  private final CommandJoystick m_stick2 = new CommandJoystick(1);

  private final BooleanSupplier m_switchSides2;
  private BooleanSupplier m_switchSides1;

  SendableChooser<Command> m_selectAuto = new SendableChooser<>();




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
    m_frontCamera.connect("10.21.2.10", 5800);
    m_backCamera.connect("10.21.2.11", 5800);

    m_driveSubsystem = new DriveSubsystem(m_frontCamera, m_backCamera, m_tags);
    m_armSubsystem = new ArmSubsystem(m_reachSubsystem, m_intakeSubsystem, m_driveSubsystem.getTracker()); 
    m_switchSides2 = () -> m_stick2.getThrottle() < 0;

    // Choose which Joystick Driver 1 wants
    if (Constants.k_xboxController) {
      m_xbox1 = new CommandXboxController(0);
      m_joystick1 = null;
    } else {
      m_joystick1 = new CommandJoystick(0);
      m_xbox1 = null;
    }

    // Shuffleboard commands
    SmartDashboard.putData("SetArmZero", new SetArmZeroCommand(m_armSubsystem));
    SmartDashboard.putData("Paradox Lights", new SetLEDCommand(m_LEDSubsystem, "idle"));
    // SmartDashboard.putData("Request Cube", new SetLEDCommand(m_LEDSubsystem, "cube"));
    // SmartDashboard.putData("Request Cone", new SetLEDCommand(m_LEDSubsystem, "cone"));

    ShuffleboardTab driverTab = Shuffleboard.getTab("Tab 4");
    driverTab.addCamera("Camera Viewer", "Front Camera", "http://10.21.2.2:1181/?action=stream").withPosition(1, 1);
    // Configure the trigger bindings
    configureBindings();
  }

  public void initialize() {
    // new SetLEDCommand(m_LEDSubsystem, "idle");
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
      m_switchSides1 = () -> m_joystick1.getThrottle() < 0;
      m_driveSubsystem.setDefaultCommand(new ArcadeDriveCommand(m_driveSubsystem, () -> m_joystick1.getX(), () -> m_joystick1.getY(), m_switchSides1));
      m_joystick1.button(1).toggleOnTrue(new IntakeCommand(m_intakeSubsystem, -0.25, false)); //intake
      m_joystick1.button(2).toggleOnTrue(new IntakeCommand(m_intakeSubsystem, 0.5, false)); //outake //0.25
      m_joystick1.button(5).toggleOnTrue(new DeliverGamePieceCommand(m_driveSubsystem, m_armSubsystem, m_reachSubsystem, () -> !m_switchSides2.getAsBoolean()));
      if (Constants.k_isCompetition) {
        m_joystick1.button(6).toggleOnTrue(new SetClawCommand(m_intakeSubsystem, IntakeSubsystem.ClawPosition.OPEN));
      } else {
        m_joystick1.button(6).toggleOnTrue(new ManualClawCommand(m_intakeSubsystem));
      }
      m_joystick1.button(7).onTrue(new BrakeOffCommand(m_driveSubsystem));
      m_joystick1.button(9). whileTrue(new ManualArmCommand(m_armSubsystem, () -> .2));
      m_joystick1.button(11). whileTrue(new ManualArmCommand(m_armSubsystem, () -> -.2));
    }

    // Driver 2
    m_stick2.button(1).toggleOnTrue(new IntakeCommand(m_intakeSubsystem, -0.2, false)); //intake
    m_stick2.button(2).toggleOnTrue(new IntakeCommand(m_intakeSubsystem, 0.5, false)); //outake //0.25
    m_stick2.button(3).onTrue(new SetArmPositionExtent(m_reachSubsystem, m_armSubsystem, Constants.k_humanPlayerStationExtent, Constants.k_humanPlayerStationAngle, m_switchSides2, 5, 0, false));
    m_stick2.button(4).whileTrue(new ManualReachCommand(m_reachSubsystem, -.3, false)); //in
    m_stick2.button(5).onTrue(new SetArmPositionExtent(m_reachSubsystem, m_armSubsystem, Constants.k_straightUpExtent, Constants.k_straightUpAngle, m_switchSides2, 0, 0, true));
    m_stick2.button(6).whileTrue(new ManualReachCommand(m_reachSubsystem, .3, false)); //out
    m_stick2.button(7).onTrue(new SetArmPositionExtent(m_reachSubsystem, m_armSubsystem, Constants.k_midConeNodeExtent, Constants.k_midConeNodeAngle, m_switchSides2, 0, 0, false));
    m_stick2.button(8).onTrue(new SetArmPositionExtent(m_reachSubsystem, m_armSubsystem, Constants.k_topConeExtent, Constants.k_topConeNodeAngle, m_switchSides2, 0, 0, false));
    m_stick2.button(9).onTrue(new SetArmPositionExtent(m_reachSubsystem, m_armSubsystem, Constants.k_midCubeNodeExtent, Constants.k_midCubeNodeAngle, m_switchSides2, 0, 0, false));
    m_stick2.button(10).toggleOnTrue(new SetClawCommand(m_intakeSubsystem, IntakeSubsystem.ClawPosition.CUBE));
    m_stick2.button(11).onTrue(new SetArmPositionExtent(m_reachSubsystem, m_armSubsystem, Constants.k_groundPickupExtent, Constants.k_groundPickupAngle, m_switchSides2, 0, 0, false));
    m_stick2.pov(0).onTrue(new ManualAdjustArmAngle(m_armSubsystem, m_switchSides2, 3));
    m_stick2.pov(180).onTrue(new ManualAdjustArmAngle(m_armSubsystem, m_switchSides2, -3));
    if (Constants.k_isCompetition) {
      m_stick2.button(12).toggleOnTrue(new SetClawCommand(m_intakeSubsystem, IntakeSubsystem.ClawPosition.CONE));
    } else {
      m_stick2.button(12).toggleOnTrue(new ManualClawCommand(m_intakeSubsystem));
    }

    // Auto Selection
    m_selectAuto.addOption("Pie | 4LS", new Auto_4LS(m_driveSubsystem, m_armSubsystem, m_intakeSubsystem));
    m_selectAuto.addOption("Nectarine | 1MA2M", new Auto_1MA2M(m_driveSubsystem, m_reachSubsystem, m_armSubsystem, m_intakeSubsystem));
    m_selectAuto.addOption("Mango | 4MS", new Auto_4MS(m_intakeSubsystem, m_reachSubsystem, m_armSubsystem, m_driveSubsystem));
    m_selectAuto.addOption("Grape | 4LBS", new Auto_4LBS(m_driveSubsystem, m_armSubsystem, m_intakeSubsystem));
    m_selectAuto.addOption("Apple | 2LA", new Auto_2LA(m_driveSubsystem, m_reachSubsystem, m_armSubsystem, m_intakeSubsystem));
    m_selectAuto.addOption("Banana | 2LA2M", new Auto_2LA2M(m_driveSubsystem, m_reachSubsystem, m_armSubsystem, m_intakeSubsystem));
    m_selectAuto.addOption("Pineapple | 9LD", new Auto_9LD(m_driveSubsystem, m_reachSubsystem, m_armSubsystem, m_intakeSubsystem));
    m_selectAuto.addOption("Strawberry | 9MD", new Auto_9MD(m_armSubsystem, m_reachSubsystem, m_intakeSubsystem, m_driveSubsystem));
    m_selectAuto.addOption("Kiwi | 9HD", new Auto_9HD(m_armSubsystem, m_reachSubsystem, m_intakeSubsystem, m_driveSubsystem));
    m_selectAuto.addOption("Orange | 9LD9M", new Auto_9LD9M(m_driveSubsystem, m_reachSubsystem, m_armSubsystem, m_intakeSubsystem));
    m_selectAuto.addOption("Papaya | 5L", new Auto_5L(m_armSubsystem, m_intakeSubsystem));
    m_selectAuto.addOption("Burrito | 1LA2MB2H", new Auto_1LA2MB2H(m_driveSubsystem, m_reachSubsystem, m_armSubsystem, m_intakeSubsystem));
    m_selectAuto.addOption("Dragonfruit | 1LA2LB3L", new Auto_1LA2LB3L(m_driveSubsystem, m_reachSubsystem, m_armSubsystem, m_intakeSubsystem));
    
    SmartDashboard.putData(m_selectAuto); 
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
  
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_selectAuto.getSelected();
  }
}
