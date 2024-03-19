package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class SetClimberToPosition extends Command {
    double position;
    Climber climber;
    double deadband = .7;
    double[] initialValue;
    

    public SetClimberToPosition(double position){
        climber = Climber.getInstance();
        this.position = position;
        addRequirements(climber);
        
    }

    @Override
    public void initialize() {
        if(climber.encoderValues()[1] > position){
            climber.lowerRight();
        }else if(climber.encoderValues()[1] < position){
            climber.increaseRightHeight().schedule();
        }
        if(climber.encoderValues()[1] > position){
            climber.lowerLeft();
        }else if(climber.encoderValues()[1] < position){
            climber.increaseLeftHeight().schedule();
        }
    }

    @Override
    public void execute(){
        SmartDashboard.putBoolean("LeftPressed", false);
        if(Math.abs(climber.encoderValues()[1] - position) < deadband){
            climber.stopRight();
        }
        if(Math.abs(climber.encoderValues()[1] - position) < deadband){
            climber.stopLeft();
        }
    }

    @Override
    public void end(boolean interrupted){
        climber.stopBoth();
    }
    @Override
    public boolean isFinished() {
        return Math.abs(climber.encoderValues()[0] - position) < deadband && Math.abs(climber.encoderValues()[1] - position) < deadband;
    }

    
}
