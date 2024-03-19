// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.BoostDrive;
import frc.robot.commands.ChangeDriveType;
import frc.robot.commands.CurveDrive;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.SetClimberToPosition;
import frc.robot.commands.SetClimberToTop;
import frc.robot.commands.ZeroClimbers;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Lightshow;
import frc.robot.subsystems.TankDriveTrain;

import java.time.Instant;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.StadiaController.Button;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
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
public class RobotContainer implements Constants{
  // Configuration Options
  final static public String kArcadeDrive = "BoostDrive";
  final static public String kBoostDrive = "BoostDrive";
  final static public String kCurveDrive = "CurveDrive";
  final static public String kDefaultDriveType = kArcadeDrive;

  // The robot's subsystems and commands are defined here...

  // Drive Commands
  private BoostDrive m_boostDrive;
  private CurveDrive m_curveDrive;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(
      kDriverControllerPort);
  private final XboxController xbox = new XboxController(1);
  public Climber climber;

  // This is a link to the lightshow subsystem
  private final Lightshow m_lightshow;
  private final TankDriveTrain m_tankDriveTrain;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    /*
     * Initialize Subsystems
     */

    // Get the lightshow subsystem
    m_lightshow = Lightshow.getInstance();
    climber = Climber.getInstance();

    // Get the drivetrain subsystem
    m_tankDriveTrain = TankDriveTrain.getInstance();

    /*
     * Configure the button bindings and commands
     */
    configureBindings();
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

    BooleanSupplier upControllerLeftC2 = () -> (xbox.getLeftY() > 0.3);
    BooleanSupplier downControllerLeftC2 = () -> (xbox.getLeftY() < -0.3);
    BooleanSupplier upControllerRightC2 = () -> (xbox.getRightY() > 0.3);
    BooleanSupplier downControllerRightC2 = () -> (xbox.getRightY() < -0.3);
    

    new POVButton(xbox, 0).onTrue(new SetClimberToTop());
    new POVButton(xbox, 180).onTrue(new ZeroClimbers());
    // new Trigger(upControllerLeftC2).onTrue(new SetClimberToTop());
    // new Trigger(downControllerLeftC2).onTrue(new ZeroClimbers());
    
    // new Trigger(upControllerLeftC2).onTrue(climber.increaseLeftHeight())
        // .onFalse(new InstantCommand(climber::stopLeft));
    // new Trigger(upControllerRightC2).onTrue(climber.increaseRightHeight())
        // .onFalse(new InstantCommand(climber::stopRight));
    // new Trigger(downControllerRightC2).onTrue(new InstantCommand(climber::lowerRight))
        // .onFalse(new InstantCommand(climber::stopRight));
    // new Trigger(downControllerLeftC2).onTrue(new InstantCommand(climber::lowerLeft))
        // .onFalse(new InstantCommand(climber::stopLeft));




    m_boostDrive = new BoostDrive(m_tankDriveTrain,
          () -> -m_driverController.getRawAxis(1),
          () -> -m_driverController.getRawAxis(4),
          () -> m_driverController.getRawAxis(3),
          () -> m_driverController.getRawAxis(2));

    m_curveDrive = new CurveDrive(m_tankDriveTrain,
          () -> -m_driverController.getRawAxis(1),
          () -> -m_driverController.getRawAxis(4),
          () -> m_driverController.getRawAxis(3),
          () -> m_driverController.getRawAxis(2));

    setDriveType(kDefaultDriveType);



    new Trigger(m_driverController.back()).onTrue(Commands.runOnce(this::toggleDriveType));


  }

  public void toggleDriveType(){
    if(m_tankDriveTrain.getDefaultCommand() != m_boostDrive){
      setDriveType(kBoostDrive);
    }
    else{
      setDriveType(kCurveDrive);
    }
  }

  public void setDriveType(String type){
    System.out.printf("Setting drive type to: %s",type);
    if (type == kArcadeDrive) {
      m_tankDriveTrain.setDefaultCommand(m_boostDrive);
      m_lightshow.setMode(Lightshow.kModeBlueRotate);;
    }
    else if (type == kBoostDrive) {
      m_tankDriveTrain.setDefaultCommand(m_boostDrive);
      m_lightshow.setMode(Lightshow.kModeBlueRotate);;
    }
    else if (type == kCurveDrive) {
      m_tankDriveTrain.setDefaultCommand(m_curveDrive);
      m_lightshow.setMode(Lightshow.kModeBrownRotate);;
    }
    else{
      System.out.println("*** WARNING *** Unrecognized drive type. Setting to default.");
      m_tankDriveTrain.setDefaultCommand(m_boostDrive);
      m_lightshow.setMode(Lightshow.kModeError);;
    }
    m_tankDriveTrain.getDefaultCommand().schedule();

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return null; // Autos.exampleAuto(m_exampleSubsystem);
  }
}
