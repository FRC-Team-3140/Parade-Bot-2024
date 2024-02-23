package frc.robot.commands.lightcommands;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Lightshow;

public class SolidColor extends Command {
    private final Lightshow lightshow;
    private final Color color;
    private final double duration;
    private long startTime;

    public SolidColor(Color color, double duration) {
        this.lightshow = Lightshow.getInstance();
        this.color = color;
        this.duration = duration;

        addRequirements(this.lightshow);
    }

    @Override
    public void initialize() {
        startTime = System.currentTimeMillis();
        Color[] colors = new Color[lightshow.getTotalLights()];
        for (int i = 0; i < colors.length; i++) {
            colors[i] = color;
        }
        lightshow.setColors(colors);
    }

    @Override
    public void execute() {}

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        if (duration == 0.0) {
            return false;
        } else {
            return System.currentTimeMillis() - startTime >= duration * 1000;
        }
    }

    @Override
    public boolean runsWhenDisabled(){
        return true;
    }
}