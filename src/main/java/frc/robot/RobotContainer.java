// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.BoostDrive;
import frc.robot.commands.ClimbDown;
import frc.robot.commands.ClimbDownLeft;
import frc.robot.commands.ClimbDownRight;
import frc.robot.commands.ClimbUp;
import frc.robot.commands.ClimbUpLeft;
import frc.robot.commands.ClimbUpRight;
import frc.robot.commands.CurveDrive;
import frc.robot.commands.autos.DriveTimed;
import frc.robot.commands.autos.TurnTimed;
import frc.robot.commands.lightcommands.SolidColor;
import frc.robot.subsystems.Lightshow;
import frc.robot.subsystems.NavXSubsystem;
import frc.robot.subsystems.DifferentialDriveSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Configuration Options
  final static public String kArcadeDrive = "BoostDrive";
  final static public String kBoostDrive = "BoostDrive";
  final static public String kCurveDrive = "CurveDrive";
  final static public String kDefaultDriveType = kArcadeDrive;

  // The robot's subsystems and commands are defined here...

  // Drive Commands
  private BoostDrive m_boostDrive;
  private CurveDrive m_curveDrive;

  // The robot's subsystems and commands are defined here...

  private final SendableChooser<Command> chooser = new SendableChooser<>();

  private final boolean kClimberEnabled = false;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);

  // This is a link to the lightshow subsystem
  private final Lightshow m_lightshow;
  private final DifferentialDriveSubsystem m_tankDriveTrain;
  private final NavXSubsystem m_navXSubsystem;

  private ClimbUpLeft m_climbUpLeft;
  private ClimbDownLeft m_climbDownLeft;
  private ClimbUpRight m_climbUpRight;
  private ClimbDownRight m_climbDownRight;
  private ClimbUp m_climbUp;
  private ClimbDown m_climbDown;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    /*
     * Initialize Subsystems
     */

    // Get the lightshow subsystem
    m_lightshow = Lightshow.getInstance();

    // Get the drivetrain subsystem
    m_tankDriveTrain = DifferentialDriveSubsystem.getInstance();

    // Get the NavX subsystem
    m_navXSubsystem = NavXSubsystem.getInstance();

    if (kClimberEnabled) {
      m_climbUpLeft = new ClimbUpLeft();
      m_climbDownLeft = new ClimbDownLeft();
      m_climbUpRight = new ClimbUpRight();
      m_climbDownRight = new ClimbDownRight();
      m_climbUp = new ClimbUp();
      m_climbDown = new ClimbDown();
    }
    /*
     * Configure the button bindings and commands
     */
    configureBindings();

    // Add commands to the autonomous command chooser
    chooser.setDefaultOption("Back Out Of Zone", new DriveTimed(m_tankDriveTrain, -0.75, 1.5));
    chooser.addOption("Rotate Left Approx 90", new TurnTimed(m_tankDriveTrain, 0.5, 0.95));
    // Put the chooser on the dashboard
    SmartDashboard.putData("Auto mode", chooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(tankDrive));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    m_boostDrive = new BoostDrive(m_tankDriveTrain,
        () -> -m_driverController.getRawAxis(1), // Left Stick Y
        () -> -m_driverController.getRawAxis(4), // Right Stick X
        () -> m_driverController.getRawAxis(3),
        () -> m_driverController.getRawAxis(2));

    m_curveDrive = new CurveDrive(m_tankDriveTrain,
        () -> -m_driverController.getRawAxis(1),
        () -> -m_driverController.getRawAxis(4),
        () -> m_driverController.getRawAxis(3),
        () -> m_driverController.getRawAxis(2));

    setDriveType(kDefaultDriveType);

    new Trigger(m_driverController.back()).onTrue(Commands.runOnce(this::toggleDriveType));

    if (kClimberEnabled) {
      new Trigger(m_driverController.y()).whileTrue(m_climbUpRight);
      new Trigger(m_driverController.x()).whileTrue(m_climbUpLeft);
      new Trigger(m_driverController.b()).whileTrue(m_climbDownRight);
      new Trigger(m_driverController.a()).whileTrue(m_climbDownLeft);
      new Trigger(m_driverController.povDown()).whileTrue(m_climbDown);
      new Trigger(m_driverController.povUp()).whileTrue(m_climbUp);
    }
    // Assuming m_driverController is a Joystick or XboxController object
  }

  public void toggleDriveType() {
    if (m_tankDriveTrain.getDefaultCommand() != m_boostDrive) {
      setDriveType(kBoostDrive);
    } else {
      setDriveType(kCurveDrive);
    }
  }

  public void setDriveType(String type) {
    System.out.printf("Setting drive type to: %s", type);
    if (type == kArcadeDrive) {
      m_tankDriveTrain.setDefaultCommand(m_boostDrive);
      m_lightshow.setDefaultCommand(new SolidColor(Lightshow.kBlue, 0.0));
      ;
    } else if (type == kBoostDrive) {
      m_tankDriveTrain.setDefaultCommand(m_boostDrive);
      m_lightshow.setDefaultCommand(new SolidColor(Lightshow.kBlue, 0.0));
      ;
    } else if (type == kCurveDrive) {
      m_tankDriveTrain.setDefaultCommand(m_curveDrive);
      m_lightshow.setDefaultCommand(new SolidColor(Lightshow.kOrange, 0.0));
      ;
    } else {
      System.out.println("*** WARNING *** Unrecognized drive type. Setting to default.");
      m_tankDriveTrain.setDefaultCommand(m_boostDrive);
      m_lightshow.setDefaultCommand(new SolidColor(Lightshow.kRed, 0.0));
      ;
    }
    m_tankDriveTrain.getDefaultCommand().schedule();
    m_lightshow.getDefaultCommand().schedule();

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return chooser.getSelected();
  }
}
