
package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;


/**
 *
 */
public class ClimbRamp extends Command {

    private final DriveTrain m_driveTrain;


    private double m_speed = 0.60;
    private double m_stop_angle = 10.0;
    private NetworkTable m_navx_table;


    public ClimbRamp(DriveTrain subsystem) {

        m_driveTrain = subsystem;
        addRequirements(m_driveTrain);

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        m_navx_table = inst.getTable("SmartDashboard").getSubTable("DataNAVX");

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        double angle = m_navx_table.getEntry("navx_filtered_pitch").getDouble(0.0);
        
        if (angle < 0) {
            m_driveTrain.arcadeDrive(m_speed, 0);
        } else {
            m_driveTrain.arcadeDrive(-m_speed, 0);

        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        double angle = m_navx_table.getEntry("navx_filtered_pitch").getDouble(0.0);
        return Math.abs(angle) < m_stop_angle;
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;

    }
}
