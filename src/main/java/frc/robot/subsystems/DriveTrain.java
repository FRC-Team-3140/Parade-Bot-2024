// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: Subsystem.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

/**
 * A drivetrain based on differential drive kinematics.
 */
public class DriveTrain extends SubsystemBase {
    public static final double kPositionConversionFactor = 1.0 / 3000.0;
    public static final double kTrackWidth = 0.6;
    public static final CANSparkBase.IdleMode idleMode = CANSparkBase.IdleMode.kBrake;


    final private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.6);
    final private DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0.0), 0.0,
            0.0);

    // The left motor group
    private CANSparkMax leftSpark9;
    private WPI_TalonSRX leftTalon4;
    private WPI_TalonSRX leftTalon6;
    private MotorControllerGroup leftMotorGroup;

    // The right motor group
    private CANSparkMax rightSpark8;
    private WPI_TalonSRX rightTalon5;
    private WPI_TalonSRX rightTalon7;
    private MotorControllerGroup rightMotorGroup;

    // The robot's drive controller
    private DifferentialDrive differentialDrive1;

    double accel_angle = 0.0;
    double angle_filtered = 0.0;

    double m_distance = 0.0;
    double m_speed = 0.0;
    double m_speed_filtered = 0.0;

    LinearFilter m_angle_filter;
    LinearFilter m_speed_filter;

    double afpc = 0.02;
    double aftc = 0.2;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    NetworkTable drivetrain_table;
    // private final DoubleEntry max_speed;

    /**
    *
    */
    public DriveTrain() {

        leftSpark9 = new CANSparkMax(9, CANSparkLowLevel.MotorType.kBrushless);
        leftTalon4 = new WPI_TalonSRX(4);
        leftTalon6 = new WPI_TalonSRX(6);
        leftMotorGroup = new MotorControllerGroup(leftSpark9, leftTalon4, leftTalon6);
        addChild("Left Motor Group", leftMotorGroup);

        rightSpark8 = new CANSparkMax(8, CANSparkLowLevel.MotorType.kBrushless);
        rightTalon5 = new WPI_TalonSRX(5);
        rightTalon7 = new WPI_TalonSRX(7);
        rightMotorGroup = new MotorControllerGroup(rightSpark8, rightTalon5, rightTalon7);
        addChild("Right Motor Group", rightMotorGroup);

        differentialDrive1 = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
        addChild("Differential Drive 1", differentialDrive1);

        differentialDrive1.setSafetyEnabled(true);
        differentialDrive1.setExpiration(0.1);
        differentialDrive1.setMaxOutput(1.0);

        // The left configuration
        leftSpark9.setIdleMode(idleMode);
        leftSpark9.setInverted(true);
        leftSpark9.burnFlash();

        leftTalon4.set(ControlMode.PercentOutput, 0.0);
        leftTalon4.configOpenloopRamp(0.2);
        leftTalon4.setInverted(false);

        leftTalon6.set(ControlMode.PercentOutput, 0.0);
        leftTalon6.configOpenloopRamp(0.2);
        leftTalon6.setInverted(false);

        // The right configuration
        rightSpark8.setIdleMode(idleMode);
        rightSpark8.setInverted(false);
        rightSpark8.burnFlash();

        rightTalon5.set(ControlMode.PercentOutput, 0.0);
        rightTalon5.configOpenloopRamp(0.2);
        rightTalon5.setInverted(true);

        rightTalon7.set(ControlMode.PercentOutput, 0.0);
        rightTalon7.configOpenloopRamp(0.2);
        rightTalon7.setInverted(true);

        // Set the current limits
        SupplyCurrentLimitConfiguration current_limit = new SupplyCurrentLimitConfiguration();
        current_limit.currentLimit = 30;
        leftTalon4.configSupplyCurrentLimit(current_limit);
        leftTalon6.configSupplyCurrentLimit(current_limit);
        rightTalon5.configSupplyCurrentLimit(current_limit);
        rightTalon7.configSupplyCurrentLimit(current_limit);

        // Set up the encoders
        leftEncoder = leftSpark9.getEncoder();
        rightEncoder = rightSpark8.getEncoder();

        // set the encorder speed conversion factor from native units to meters per
        // second.
        leftEncoder.setVelocityConversionFactor(kPositionConversionFactor);
        rightEncoder.setVelocityConversionFactor(kPositionConversionFactor);

        // set the encorder position conversion factor from native units to meters.
        leftEncoder.setPositionConversionFactor(kPositionConversionFactor);
        rightEncoder.setPositionConversionFactor(kPositionConversionFactor);

        // TODO: Switch network tables to Comms3140
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        drivetrain_table = inst.getTable("SmartDashboard").getSubTable("DriveTrain");

        // Create persistant configuration options in network table.
        aftc = drivetrain_table.getEntry("angle_filter_time_const").getNumber(0.2).doubleValue();
        afpc = drivetrain_table.getEntry("angle_filter_period_const").getNumber(0.02).doubleValue();

        drivetrain_table.getEntry("angle_filter_time_const").setNumber(aftc);
        drivetrain_table.getEntry("angle_filter_period_const").setNumber(afpc);

        drivetrain_table.getEntry("angle_filter_time_const").setPersistent();
        drivetrain_table.getEntry("angle_filter_period_const").setPersistent();

        m_angle_filter = LinearFilter.singlePoleIIR(aftc, afpc);
        m_speed_filter = LinearFilter.singlePoleIIR(0.5, 0.02);
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run

        checkNetworkTableChanges();

        if (angle_filtered > 15)
            angle_filtered = 15;
        if (angle_filtered < -15)
            angle_filtered = -15;

        m_distance = leftEncoder.getPosition();
        m_speed = leftEncoder.getVelocity();

        odometry.update(Rotation2d.fromDegrees(0.0), leftEncoder.getPosition(), rightEncoder.getPosition());

        m_speed_filtered = m_speed_filter.calculate(m_speed);

        drivetrain_table.getEntry("distance").setDouble(m_distance);
        drivetrain_table.getEntry("speed").setDouble(m_speed);
        drivetrain_table.getEntry("speed_filtered").setDouble(m_speed_filtered);
    }

    /**
     * TODO: Remove this method
     * @return the m_distance
     */
    @Deprecated
    private void checkNetworkTableChanges() {
        // Check for a change in the angle filter values
        double new_tc = drivetrain_table.getEntry("angle_filter_time_const").getNumber(0.2).doubleValue();
        double new_pc = drivetrain_table.getEntry("angle_filter_period_const").getNumber(0.02).doubleValue();
        if (new_tc != aftc || new_pc != afpc) {
            aftc = new_tc;
            afpc = new_pc;
            m_angle_filter = LinearFilter.singlePoleIIR(aftc, afpc);
            System.out.printf("Updating angle filter values. time=%.3f period=%.3f \n", aftc, afpc);
        }

        angle_filtered = m_angle_filter.calculate(accel_angle);

        drivetrain_table.getEntry("angle_filtered").setDouble(angle_filtered);
        drivetrain_table.getEntry("speed_filtered").setDouble(m_speed_filtered);

    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    public void arcadeDrive(double xSpeed, double zRotation) {
        drivetrain_table.getEntry("arcade_xspeed").setNumber(xSpeed);
        drivetrain_table.getEntry("arcade_zrotation").setNumber(zRotation);

        differentialDrive1.arcadeDrive(xSpeed, zRotation);
    }

    public void curveDrive(double xSpeed,double zRotation){
        differentialDrive1.curvatureDrive(xSpeed , zRotation, false);
    }

    public double getSpeedFiltered() {
        return m_speed_filtered;
    }

    public double getSpeed() {
        return m_speed;
    }

    public double getPosition() {
        return m_distance;
    }
    // Put methods for controlling this subsystem
    // here. Call these from Commands.

    /**
     * Reset the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
    }

    /**
     * Get the current pose of the robot using odometry.
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Reset the odometry to the specified pose.
     * 
     * @param pose
     */
    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(Rotation2d.fromDegrees(0.0), leftEncoder.getPosition(),
                rightEncoder.getPosition(), pose);
    }

    /**
     * Reset the odometry to zero.
     */
    public void resetOdometry() {
        resetOdometry(new Pose2d());
    }

}
