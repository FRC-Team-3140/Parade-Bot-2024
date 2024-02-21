package frc.robot.subsystems;

import edu.wpi.first.hal.can.CANStatus;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import java.util.HashMap;
import java.util.Map;

public class SystemMonitor extends SubsystemBase {
    private static SystemMonitor instance = null;
    private Map<String, MotorStats> motorStats;
    private Map<String, CANSparkMax> motors;
    private final NetworkTable motorMonitorTable;
    private final NetworkTable canBusTable;

    private static final double HIGH_CURRENT_THRESHOLD = 30.0; // Adjust as needed
    private static final double HIGH_TEMP_THRESHOLD = 80.0; // Adjust as needed

    class MotorStats {
        private double maxCurrent = 0.0;
        private double maxSpeed = 0.0;
        private double totalRevolutions = 0.0;
        private double currentPosition = 0.0;
        private double currentSpeed = 0.0;
        private double lastPosition = 0.0;

        public void updateMaxCurrent(double current) {
            if (current > maxCurrent) {
                maxCurrent = current;
            }
        }

        public void updateMaxSpeed(double speed) {
            if (speed > maxSpeed) {
                maxSpeed = speed;
            }
        }

        public void updateTotalRevolutions(double position) {
            totalRevolutions += Math.abs(position - lastPosition);
            lastPosition = position;
        }

        public void updateCurrentPosition(double position) {
            currentPosition = position;
        }

        public void updateCurrentSpeed(double speed) {
            currentSpeed = speed;
        }

        public double getMaxCurrent() {
            return maxCurrent;
        }

        public double getMaxSpeed() {
            return maxSpeed;
        }

        public double getTotalRevolutions() {
            return totalRevolutions;
        }

        public double getCurrentPosition() {
            return currentPosition;
        }

        public double getCurrentSpeed() {
            return currentSpeed;
        }
    }

    private SystemMonitor() {
        motors = new HashMap<>();
        motorStats = new HashMap<>();
        motorMonitorTable = NetworkTableInstance.getDefault().getTable("SystemMonitor");
      canBusTable = motorMonitorTable.getSubTable("CAN Bus");
   }

    public static SystemMonitor getInstance() {
        if (instance == null) {
            instance = new SystemMonitor();
        }
        return instance;
    }

    public void registerMotor(String name, CANSparkMax motor) {
        motors.put(name, motor);
        motorStats.put(name, new MotorStats());
    }

    public void monitorMotor(String name) {
        CANSparkMax motor = motors.get(name);
        MotorStats stats = motorStats.get(name);
        NetworkTable motorTable = motorMonitorTable.getSubTable(name);

        double current = motor.getOutputCurrent();
        double temp = motor.getMotorTemperature();
        double speed = motor.get();
        int faults = motor.getFaults();

        // Update stats
        stats.updateMaxCurrent(current);
        stats.updateMaxSpeed(speed);
        stats.updateTotalRevolutions(motor.getEncoder().getPosition());

        if (current > HIGH_CURRENT_THRESHOLD) {
            motorTable.getEntry("Warning").setString("High current");
        }

        if (temp > HIGH_TEMP_THRESHOLD) {
            motorTable.getEntry("Warning").setString("High temperature");
        }

        if (faults > 0) {
            motorTable.getEntry("Warning").setString("Motor fault");
        }

        // Post motor data and stats to NetworkTable
        motorTable.getEntry("Current").setDouble(current);
        motorTable.getEntry("Temperature").setDouble(temp);
        motorTable.getEntry("Faults").setDouble(faults);
        motorTable.getEntry("Max Current").setDouble(stats.getMaxCurrent());
        motorTable.getEntry("Max Speed").setDouble(stats.getMaxSpeed());
        motorTable.getEntry("Total Revolutions").setDouble(stats.getTotalRevolutions());
    }

    public void monitorAllMotors() {
        for (String name : motors.keySet()) {
            monitorMotor(name);
        }
    }

    public void scanCANBus() {
        CANStatus canStatus = RobotController.getCANStatus();

        canBusTable.getEntry("Percent Bus Utilization").setDouble(canStatus.percentBusUtilization);
        canBusTable.getEntry("Bus Off Count").setDouble(canStatus.busOffCount);
        canBusTable.getEntry("TX Full Count").setDouble(canStatus.txFullCount);
        canBusTable.getEntry("Receive Error Count").setDouble(canStatus.receiveErrorCount);
        canBusTable.getEntry("Transmit Error Count").setDouble(canStatus.transmitErrorCount);    
    }

    @Override
    public void periodic() {
        monitorAllMotors();
        scanCANBus();

    }
}