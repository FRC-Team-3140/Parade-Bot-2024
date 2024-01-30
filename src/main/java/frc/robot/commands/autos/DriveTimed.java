package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DifferentialDriveSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class DriveTimed extends Command {
    private final DifferentialDriveSubsystem drive;
    private final double speed;
    private final double time;
    private final Timer timer = new Timer();

    public DriveTimed(DifferentialDriveSubsystem drive, double speed, double time) {
        this.drive = drive;
        this.speed = speed;
        this.time = time;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        drive.arcadeDrive(speed, 0);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stopMotor();
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= time;
    }
}