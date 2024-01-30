package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DifferentialDriveSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class TurnTimed extends Command {
    private final DifferentialDriveSubsystem drive;
    private final double zRotation;
    private final double time;
    private final Timer timer = new Timer();

    public TurnTimed(DifferentialDriveSubsystem drive, double zRotation, double time) {
        this.drive = drive;
        this.zRotation = zRotation;
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
        drive.arcadeDrive(0, zRotation);
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