package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AcmeFlagWaver2000 extends SubsystemBase {

    private final TalonSRX waver;
    private double speed = 0.0;
    private double max_speed = 0.4;

    public AcmeFlagWaver2000(){

        waver = new TalonSRX(12);

        SupplyCurrentLimitConfiguration current_limit = new SupplyCurrentLimitConfiguration();
        current_limit.currentLimit = 10;

        waver.configSupplyCurrentLimit(current_limit);

        // set the current limit
    }

    @Override
    public void periodic() {

        waver.set(ControlMode.PercentOutput, speed);
        
    }

    public void wave() {
        speed = max_speed;

    }

    public void stop(){
        speed = 0.0;
    }


    
}
