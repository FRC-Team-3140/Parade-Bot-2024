
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

/**
 *
 */
public class ParkingBrake extends Command {

    private final DriveTrain m_driveTrain;

    private double m_stop_position = 0.0;
    private double m_max_speed = 0.7;

    // Tune this until the robot stops nicely while on the incline
    PIDController pid = new PIDController(8.00, 0.00, 0.00);

    public ParkingBrake(DriveTrain subsystem) {

        m_driveTrain = subsystem;
        addRequirements(m_driveTrain);

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_stop_position = m_driveTrain.getPosition();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        double position = m_driveTrain.getPosition();
        double speed = pid.calculate(position - m_stop_position);

        speed = Math.min(Math.max(speed, -m_max_speed), m_max_speed); // limit the speed

        m_driveTrain.arcadeDrive(speed, 0);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;
    }
}
