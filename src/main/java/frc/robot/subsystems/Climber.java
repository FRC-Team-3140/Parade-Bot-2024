package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkBase.IdleMode;


public class Climber extends SubsystemBase {
    private static final int kLeftClimberCanId = 9;
    private static final int kRightClimberCanId = 8;

    private final CANSparkMax m_leftClimber;
    private final CANSparkMax m_rightClimber;

    private static Climber instance;

    private Climber() {
        m_leftClimber = new CANSparkMax(kLeftClimberCanId, MotorType.kBrushless);
        m_rightClimber = new CANSparkMax(kRightClimberCanId, MotorType.kBrushless);

        m_leftClimber.setIdleMode(IdleMode.kBrake);
        m_rightClimber.setIdleMode(IdleMode.kBrake);

    }

    public static Climber getInstance() {
        if (instance == null) {
            instance = new Climber();
        }
        return instance;
    }

    public void climb(double speed) {
        m_leftClimber.set(speed);
        m_rightClimber.set(speed);
    }

    public void stop() {
        m_leftClimber.stopMotor();
        m_rightClimber.stopMotor();
    }
}