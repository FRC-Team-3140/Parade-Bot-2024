package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DifferentialDriveSubsystem;
import frc.robot.subsystems.NavXSubsystem;

public class DirectionDrive extends Command {
    private final XboxController m_controller;
    private final DifferentialDriveSubsystem m_drivetrain;
    private final NavXSubsystem m_navx;
    private double targetAngle = 0;
    private boolean isDpadPressed = false;
    private final double deadZone = 0.2;
    private final double fastTurnSpeed = 0.5;
    private final double slowTurnSpeed = 0.3;
    private final double turningDriveSpeed = 0.8;
    private final double fullDriveSpeed = 1.0;
    private final double minTurnThreshold = 2.0;
    private final double fastTurnThreshold = 8.0;

    private final NetworkTable m_table;

    public DirectionDrive(XboxController controller) {
        m_controller = controller;
        m_drivetrain = DifferentialDriveSubsystem.getInstance();
        m_navx = NavXSubsystem.getInstance();

        // Create a network table subtable for this command to help with debugging
        m_table = NetworkTableInstance.getDefault().getTable("DirectionDrive"); 

        addRequirements(m_drivetrain);
    }

    @Override
    public void execute() {
        double leftStickY = -m_controller.getRawAxis(XboxController.Axis.kLeftY.value);

        double rightStickX = m_controller.getRawAxis(XboxController.Axis.kRightX.value);
        double rightStickY = m_controller.getRawAxis(XboxController.Axis.kRightY.value);

        double rightStickAngle = Math.toDegrees(Math.atan2(rightStickY, rightStickX));
        double rightStickDistance = Math.sqrt(rightStickX*rightStickX + rightStickY*rightStickY);

        int dpadAngle = m_controller.getPOV();

        // Update the network table with the current control values
        m_table.getEntry("leftStickY").setDouble(leftStickY);
        m_table.getEntry("rightStickX").setDouble(rightStickX);
        m_table.getEntry("rightStickY").setDouble(rightStickY);
        m_table.getEntry("rightStickAngle").setDouble(rightStickAngle);
        m_table.getEntry("rightStickDistance").setDouble(rightStickDistance);
        m_table.getEntry("dpadAngle").setDouble(dpadAngle);

        if (Math.abs(rightStickX) > deadZone) {
            targetAngle = rightStickX * 180;
        }

        // If the dpad is pressed, set the target angle to the dpad angle
        if (dpadAngle != -1 && !isDpadPressed) {
            targetAngle = dpadAngle;
            isDpadPressed = true;
        } else if (dpadAngle == -1) {
            isDpadPressed = false;
        }

        double currentAngle = m_navx.getYaw().getDegrees(); // the angle in degrees        
        double error = targetAngle - currentAngle;
        error = ((error + 180) % 360) - 180;
        // Update the network table with the current error
        m_table.getEntry("currentAngle").setDouble(currentAngle);
        m_table.getEntry("targetAngle").setDouble(targetAngle);
        m_table.getEntry("error").setDouble(error);

        if (Math.abs(error) < minTurnThreshold) {
            m_drivetrain.arcadeDrive(leftStickY * fullDriveSpeed, 0);
        } else {
            double errorMagnitude = Math.abs(error);
            double turnSpeed;

            if (errorMagnitude > fastTurnThreshold) {
                turnSpeed = Math.copySign(fastTurnSpeed, error);
            } else {
                turnSpeed = Math.copySign(slowTurnSpeed, error);
            }

            m_drivetrain.arcadeDrive(leftStickY * turningDriveSpeed, turnSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_drivetrain.arcadeDrive(0, 0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
