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

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * A drivetrain based on differential drive kinematics.
 */
public class DifferentialDriveSubsystem extends SubsystemBase {

    private static final int kLeftSparkCANId = 9;
    private static final int kLeftTalon1CANId = 4;
    private static final int kLeftTalon2CANId = 6;

    private static final int kRightSparkCANId = 8;
    private static final int kRightTalon1CANId = 5;
    private static final int kRightTalon2CANId = 7;

    private static DifferentialDriveSubsystem instance = null;

    public static final double kPositionConversionFactor = 3.0/79.0; // estimated over a 3 meter test
    public static final double kTrackWidth = 0.6;
    // Calibarating track width by wheel distance for 360
    //  left    right
    // 1.673  -1.989
    //  1.603   -2.652
    // 1.604   -2.062
    // 1.991   -2.215

    public static final CANSparkBase.IdleMode idleMode = CANSparkBase.IdleMode.kBrake;

    final private DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(0.6);
    final private DifferentialDriveOdometry master_odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(0.0), 0.0,
            0.0);

    // The left motor group
    private CANSparkMax leftSpark;
    private WPI_TalonSRX leftTalon1;
    private WPI_TalonSRX leftTalon2;
    private MotorControllerGroup leftMotorGroup;

    // The right motor group
    private CANSparkMax rightSpark;
    private WPI_TalonSRX rightTalon1;
    private WPI_TalonSRX rightTalon2;
    private MotorControllerGroup rightMotorGroup;

    // The robot's drive controller
    private DifferentialDrive differentialDriveControl;

    double m_distance = 0.0;
    double m_speed = 0.0;
    double m_speed_filtered = 0.0;

    private final RelativeEncoder leftEncoder;
    private final RelativeEncoder rightEncoder;

    NetworkTable drivetrain_table;

    /**
     * Get the instance of the drivetrain.
     * 
     * @return the instance of the drivetrain.
     */
    public static DifferentialDriveSubsystem getInstance() {
        if (instance == null) {
            instance = new DifferentialDriveSubsystem();
        }
        return instance;
    }

    /**
     * Create a new instance of the drivetrain.
     */
    private DifferentialDriveSubsystem() {

        leftSpark = new CANSparkMax(kLeftSparkCANId, CANSparkLowLevel.MotorType.kBrushless);
        leftTalon1 = new WPI_TalonSRX(kLeftTalon1CANId);
        leftTalon2 = new WPI_TalonSRX(kLeftTalon2CANId);
        leftMotorGroup = new MotorControllerGroup(leftSpark, leftTalon1, leftTalon2);
        addChild("Left Motor Group", leftMotorGroup);

        rightSpark = new CANSparkMax(kRightSparkCANId, CANSparkLowLevel.MotorType.kBrushless);
        rightTalon1 = new WPI_TalonSRX(kRightTalon1CANId);
        rightTalon2 = new WPI_TalonSRX(kRightTalon2CANId);
        rightMotorGroup = new MotorControllerGroup(rightSpark, rightTalon1, rightTalon2);
        addChild("Right Motor Group", rightMotorGroup);

        SystemMonitor.getInstance().registerMotor("LeftDrive", leftSpark);
        SystemMonitor.getInstance().registerMotor("RightDrive", rightSpark);
        
        differentialDriveControl = new DifferentialDrive(leftMotorGroup, rightMotorGroup);
        addChild("Differential Drive 1", differentialDriveControl);

        differentialDriveControl.setSafetyEnabled(true);
        differentialDriveControl.setExpiration(0.1);
        differentialDriveControl.setMaxOutput(1.0);

        // The left configuration
        leftSpark.setIdleMode(idleMode);
        leftSpark.setInverted(true);
        leftSpark.burnFlash();

        leftTalon1.set(ControlMode.PercentOutput, 0.0);
        leftTalon1.configOpenloopRamp(0.2);
        leftTalon1.setInverted(false);

        leftTalon2.set(ControlMode.PercentOutput, 0.0);
        leftTalon2.configOpenloopRamp(0.2);
        leftTalon2.setInverted(false);

        // The right configuration
        rightSpark.setIdleMode(idleMode);
        rightSpark.setInverted(false);
        rightSpark.burnFlash();

        rightTalon1.set(ControlMode.PercentOutput, 0.0);
        rightTalon1.configOpenloopRamp(0.2);
        rightTalon1.setInverted(true);

        rightTalon2.set(ControlMode.PercentOutput, 0.0);
        rightTalon2.configOpenloopRamp(0.2);
        rightTalon2.setInverted(true);

        // Set the current limits
        SupplyCurrentLimitConfiguration current_limit = new SupplyCurrentLimitConfiguration();
        current_limit.currentLimit = 30;
        leftTalon1.configSupplyCurrentLimit(current_limit);
        leftTalon2.configSupplyCurrentLimit(current_limit);
        rightTalon1.configSupplyCurrentLimit(current_limit);
        rightTalon2.configSupplyCurrentLimit(current_limit);

        // Set up the encoders
        leftEncoder = leftSpark.getEncoder();
        rightEncoder = rightSpark.getEncoder();

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
    }

    /**
     * This method is called once per scheduler run to update the robot's odometry
     * and publish the current speed and distance to a NetworkTable.
     * 
     * It calculates the average distance and speed from the left and right
     * encoders,
     * updates the odometry with the current heading and distances,
     * and publishes the speed and distance to the "drivetrain" NetworkTable.
     */
    @Override
    public void periodic() {
        // Get the current position from each encoder
        double leftDistance = leftEncoder.getPosition();
        double rightDistance = rightEncoder.getPosition();

        // Calculate the average distance
        m_distance = (leftDistance + rightDistance) / 2;

        // Get the current speed from each encoder
        double leftSpeed = leftEncoder.getVelocity();
        double rightSpeed = rightEncoder.getVelocity();

        // Calculate the average speed
        m_speed = (leftSpeed + rightSpeed) / 2;

        // Update the odometry
        Rotation2d gyroAngle = Rotation2d.fromDegrees(0.0); // Replace with actual gyro angle
        master_odometry.update(gyroAngle, leftDistance, rightDistance);

        // Publish the current speed and distance to the "drivetrain" NetworkTable
        drivetrain_table.getEntry("distance").setDouble(m_distance);
        drivetrain_table.getEntry("speed").setDouble(m_speed);

        //send the encoder counts to the network table
        drivetrain_table.getEntry("leftEncoderPosition").setDouble(leftEncoder.getPosition());
        drivetrain_table.getEntry("rightEncoderPosition").setDouble(rightEncoder.getPosition());

        // send the odometry pose to the network table
        drivetrain_table.getEntry("x").setDouble(master_odometry.getPoseMeters().getX());
        drivetrain_table.getEntry("y").setDouble(master_odometry.getPoseMeters().getY());
        drivetrain_table.getEntry("heading").setDouble(master_odometry.getPoseMeters().getRotation().getDegrees());

        // The following line feeds the safety watchdog manually, which may be unsafe
        // differentialDriveControl.feed();
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

    }

    /**
     * Drives the robot using arcade drive control scheme.
     * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
     */
    public void arcadeDrive(double xSpeed, double zRotation) {
        differentialDriveControl.arcadeDrive(xSpeed, zRotation);
    }

    /**
     * Drives the robot using curvature drive control scheme.
     * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param zRotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
     */
    public void curveDrive(double xSpeed, double zRotation) {
        differentialDriveControl.curvatureDrive(xSpeed, zRotation, false);
    }

    /**
     * Drives the robot using tank drive control scheme.
     * @param leftSpeed The robot's left side speed along the X axis [-1.0..1.0]. Forward is positive.
     * @param rightSpeed The robot's right side speed along the X axis [-1.0..1.0]. Forward is positive.
     */
    public void tankDrive(double leftSpeed, double rightSpeed) {
        differentialDriveControl.tankDrive(leftSpeed, rightSpeed);
    }

    /**
     * Stops the robot by stopping the DifferentialDrive.
     */
    public void stopMotor() {
        differentialDriveControl.stopMotor();
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

    /**
     * Get the current pose of the robot.
     *
     * @return The pose of the robot in meters.
     */
    public Pose2d getPose() {
        return master_odometry.getPoseMeters();
    }

    /**
     * Reset the drive encoders to currently read a position of 0.
     */
    public void resetEncoders() {
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
    }

}
