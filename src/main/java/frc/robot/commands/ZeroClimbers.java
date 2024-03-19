package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class ZeroClimbers extends Command {
    Climber climber = Climber.getInstance();
    double deadband = .001;
    public ZeroClimbers(){
        addRequirements(climber);
    }
    @Override
    public void initialize() {
        climber.lowerBoth();
    }
    @Override
    public boolean isFinished() {
        return climber.bothLimitSwitchesPressed();
    }
}
