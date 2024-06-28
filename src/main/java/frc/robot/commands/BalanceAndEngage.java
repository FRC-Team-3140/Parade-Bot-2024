
package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class BalanceAndEngage extends Command {

    private final DriveTrain m_driveTrain;

    private NetworkTable m_navx_table;

    private double m_stop_position = 0.0;
    private double m_max_power = 0.7;

    private int m_count = 0;

    // Compute how much the angle is changing
    double last_angle = 0.0;

    // Tune this until the robot stops nicely while on the incline
    PIDController pid = new PIDController(8.00, 0.00, 0.00);

    public BalanceAndEngage(DriveTrain subsystem) {

        m_driveTrain = subsystem;
        addRequirements(m_driveTrain);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        m_navx_table = inst.getTable("SmartDashboard").getSubTable("DataNAVX");

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
        double pitch = m_navx_table.getEntry("navx_filtered_pitch").getDouble(0.0);
        m_count += 1;
        if (m_count % 100 == 0) {
            if (pitch > 2.0)
                m_stop_position -= 0.1;
            else if (pitch < -2.0)
                m_stop_position += 0.1;
        }

        double power = pid.calculate(position - m_stop_position);

        power = Math.min(Math.max(power, -m_max_power), m_max_power); // limit the speed

        m_driveTrain.arcadeDrive(power, 0);
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
