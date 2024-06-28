// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class Balance_Test_v2 extends CommandBase {
  private final DriveTrain m_DriveTrain;
  
  private final double max_speed = 0.55;

  private PIDController pidController = new PIDController(max_speed / 10, 0.02, 0.04);

  /** Creates a new Balance_Test_v2. */
  public Balance_Test_v2(DriveTrain subsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_DriveTrain = subsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = m_DriveTrain.getAngleFiltered();
    double speed = pidController.calculate(-angle);

    speed = Math.min(Math.max(speed, -max_speed), max_speed);

    m_DriveTrain.arcadeDrive(speed, 0);
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
