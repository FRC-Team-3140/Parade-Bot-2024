package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


/** 
 * This is marked as untested.  There needs to be a warning when it is used
 * 
 * @deprecated Untested. This can be removed once it has been tested and PID values are set.
 */
public class IntakeSubsystem extends SubsystemBase {
    private static final int kMotorCanId = 20; // Replace 0 with your motor's CAN ID
    private static final int kSwitchDioPort = 0; // Replace 0 with your switch's DIO port
    private static final double kP = 0.0; // Replace 0.0 with your PID P constant
    private static final double kI = 0.0; // Replace 0.0 with your PID I constant
    private static final double kD = 0.0; // Replace 0.0 with your PID D constant

    private static IntakeSubsystem instance;
    private final CANSparkMax m_intakeMotor;
    private final DigitalInput m_intakeSwitch;
    private final PIDController m_pidController;

    private IntakeSubsystem() {
        m_intakeMotor = new CANSparkMax(kMotorCanId, MotorType.kBrushless);
        m_intakeSwitch = new DigitalInput(kSwitchDioPort);
        m_pidController = new PIDController(kP, kI, kD);
    }

    /**
     * Returns the singleton instance of the IntakeSubsystem.
     * @return The singleton instance of the IntakeSubsystem.
     */
    public static IntakeSubsystem getInstance() {
        if (instance == null) {
            instance = new IntakeSubsystem();
        }
        return instance;
    }

    /**
     * Starts the intake motor at a speed controlled by the PID controller.
     * @param speed The target speed.
     */
    public void startMotor(double speed) {
        double output = m_pidController.calculate(m_intakeMotor.getEncoder().getVelocity(), speed);
        m_intakeMotor.set(output);
    }

    /**
     * Stops the intake motor.
     */
    public void stopMotor() {
        m_intakeMotor.stopMotor();
    }

    /**
     * Checks if a game piece has been loaded.
     * @return True if a game piece has been loaded, false otherwise.
     */
    public boolean isGamePieceLoaded() {
        return !m_intakeSwitch.get(); // Assumes switch is normally open
    }
}