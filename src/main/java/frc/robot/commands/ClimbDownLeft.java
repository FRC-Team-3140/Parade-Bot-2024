package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

 public class ClimbDownLeft extends Command  {
    private final Climber climber;

    public ClimbDownLeft() {
        climber = Climber.getInstance();
        addRequirements(climber); // This command requires the Climber subsystem
    }

    @Override
    public void initialize() {
        // Code to run when the command is initially scheduled
    }

    @Override
    public void execute() {
        // Code to run every time the scheduler runs while the command is scheduled
        climber.climb(-1.0,0.0); // Replace 1.0 with your desired climb speed
    }

    @Override
    public void end(boolean interrupted) {
        // Code to run when the command ends or is interrupted
        climber.stop();
    }

    @Override
    public boolean isFinished() {
        // Code to determine when this command should end
        return false; // This command never ends by itself
    }
}