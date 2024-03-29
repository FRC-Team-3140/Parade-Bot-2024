// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.TankDriveTrain;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class CurveDrive extends Command {
  private final TankDriveTrain m_drivetrain;
  private final Supplier<Double> m_xaxisSpeedSupplier;
  private final Supplier<Double> m_zaxisRotateSupplier;
  private final Supplier<Double> m_boostAxisSupplier;
  private final Supplier<Double> m_slowAxisSupplier;

  double speed_normal = 0.40;
  double speed_max = 1.0;
  double speed_min = 0.30;
  double turn_norm = 0.8;
  double turn_max = 1.0;
  double turn_min = 0.7;

  /**
   * Creates a new ArcadeDrive. This command will drive your robot according to the speed supplier
   * lambdas. This command does not terminate.
   *
   * @param drivetrain The drivetrain subsystem on which this command will run
   * @param xaxisSpeedSupplier Lambda supplier of forward/backward speed
   * @param zaxisRotateSupplier Lambda supplier of rotational speed
   */
  public CurveDrive(
      TankDriveTrain drivetrain,
      Supplier<Double> xaxisSpeedSupplier,
      Supplier<Double> zaxisRotateSupplier,
      Supplier<Double> boostAxisSupplier,
      Supplier<Double> slowAxisSupplier) {
    m_drivetrain = drivetrain;
    m_xaxisSpeedSupplier = xaxisSpeedSupplier;
    m_zaxisRotateSupplier = zaxisRotateSupplier;
    m_boostAxisSupplier = boostAxisSupplier;
    m_slowAxisSupplier = slowAxisSupplier;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double fast = Math.min(Math.max(m_boostAxisSupplier.get(),0.0),1.0);
    double slow = Math.min(Math.max(m_slowAxisSupplier.get(),0.0),1.0);
    double tfast = Math.min(Math.max(m_boostAxisSupplier.get(),0.0),1.0);
    double tslow = Math.min(Math.max(m_slowAxisSupplier.get(),0.0),1.0);
    double boost = speed_normal + (speed_max-speed_normal)*fast - (speed_normal-speed_min)*slow;
    
    m_drivetrain.curveDrive(boost*m_xaxisSpeedSupplier.get(),.45*m_zaxisRotateSupplier.get());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
