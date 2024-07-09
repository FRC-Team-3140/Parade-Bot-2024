// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.


package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrainOld;


/**
 *
 */
public class PongDrive extends Command {

    private final DriveTrainOld m_driveTrain;

    double m_angle = 0.0;
    double m_max_speed = 1.0;
    double m_max_rotation_speed = 1.0;
    double m_stop_position = 0.0;

    final NetworkTable m_DataNAVX;
    // Tune this until the robot stops nicely while on the incline
    PIDController speed_pid = new PIDController(3.00, 0.00, 0.00);
    double control_sensitivity = 0.25;

    PIDController yawController = new PIDController(0.0655, 0, 0);

    Supplier<Double> m_control_axis;

    public PongDrive(DriveTrainOld subsystem, double angle, Supplier<Double> xaxisSpeedSupplier) {


        m_angle = angle;

        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        m_DataNAVX = inst.getTable("SmartDashboard").getSubTable("DataNAVX");

        yawController.enableContinuousInput(-180, 180);

        m_control_axis = xaxisSpeedSupplier;

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
        double yaw = m_DataNAVX.getEntry("navx_yaw").getDouble(m_angle); // if this is angle then the robot will not
                                                                         // turn by default

        yawController.setSetpoint(m_angle);

        // yawController.

        double rotation_speed = yawController.calculate(-yaw);

        rotation_speed = Math.min(Math.max(rotation_speed, -m_max_rotation_speed), m_max_rotation_speed);

        double position = m_driveTrain.getPosition();

        if (Math.abs(m_control_axis.get()) > 0.25) {
            m_stop_position = position + control_sensitivity * m_control_axis.get();
        }

        double speed = speed_pid.calculate(position - m_stop_position);

        speed = Math.min(Math.max(speed, -m_max_speed), m_max_speed); // limit the speed

        m_driveTrain.arcadeDrive(speed, rotation_speed);

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