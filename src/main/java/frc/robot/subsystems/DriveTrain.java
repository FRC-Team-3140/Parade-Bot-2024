// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrain extends SubsystemBase {
  public static final double kPositionConversionFactor = 1.0 / 3000.0;
  public static final double kTrackWidth = 0.6;
  public static final IdleMode kIdleMode = IdleMode.kBrake;

  // final private DifferentialDriveOdometry odometry = new
  // DifferentialDriveOdometry(Rotation2d.fromDegrees(0.0), 0.0, 0.0);

  // The left motor group
  // leftTalonLeader
  private WPI_TalonSRX leftTalonLeader;

  // leftTalonFollower
  private WPI_TalonSRX leftTalonFollower;

  // leftSparkMaxFollower
  private CANSparkMax leftSparkMaxFollower;

  // The right motor group
  // rightTalonLeader
  private WPI_TalonSRX rightTalonLeader;

  // rightSparkMaxFollower
  private CANSparkMax rightSparkMaxFollower;

  // rightTalonFollower
  private WPI_TalonSRX rightTalonFollower;

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

  // private BuiltInAccelerometer accelerometer;

  private final RelativeEncoder leftEncoder;
  private final RelativeEncoder rightEncoder;

  NetworkTable drivetrain_table;

  private void factoryReset() {
    // SparkMaxes
    leftSparkMaxFollower.restoreFactoryDefaults(true);
    rightSparkMaxFollower.restoreFactoryDefaults(true);

    leftSparkMaxFollower.burnFlash();
    rightSparkMaxFollower.burnFlash();

    // Talons
    leftTalonLeader.configFactoryDefault();
    leftTalonFollower.configFactoryDefault();
    rightTalonLeader.configFactoryDefault();
    rightTalonFollower.configFactoryDefault();
  }

  private void setIdleModes() {
    // The left configuration
    leftSparkMaxFollower.setIdleMode(kIdleMode);
    leftSparkMaxFollower.setInverted(true);
    leftSparkMaxFollower.burnFlash();

    leftTalonLeader.configOpenloopRamp(0.2);
    leftTalonLeader.setInverted(false);
    leftTalonFollower.follow(leftTalonLeader);

    leftSparkMaxFollower.follow(CANSparkMax.ExternalFollower.kFollowerDisabled, leftTalonLeader.getDeviceID());

    // The right configuration
    rightSparkMaxFollower.setIdleMode(IdleMode.kCoast);
    rightSparkMaxFollower.setInverted(false);
    rightSparkMaxFollower.burnFlash();

    rightTalonLeader.configOpenloopRamp(0.2);
    rightTalonLeader.setInverted(true);
    rightTalonFollower.follow(rightTalonLeader);

    leftSparkMaxFollower.follow(CANSparkMax.ExternalFollower.kFollowerDisabled, rightTalonLeader.getDeviceID());
  }

  /** Creates a new DriveTrain. */
  public DriveTrain() {
    leftSparkMaxFollower = new CANSparkMax(9, MotorType.kBrushless);
    leftTalonLeader = new WPI_TalonSRX(4);
    leftTalonFollower = new WPI_TalonSRX(6);

    rightSparkMaxFollower = new CANSparkMax(8, MotorType.kBrushless);
    rightTalonLeader = new WPI_TalonSRX(5);
    rightTalonFollower = new WPI_TalonSRX(7);

    factoryReset();
    setIdleModes();

    differentialDrive1 = new DifferentialDrive(leftTalonLeader, rightTalonLeader);

    differentialDrive1.setSafetyEnabled(true);
    differentialDrive1.setExpiration(0.1);
    differentialDrive1.setMaxOutput(1.0);

    // Set the current limits
    SupplyCurrentLimitConfiguration current_limit = new SupplyCurrentLimitConfiguration();
    current_limit.currentLimit = 30;
    leftTalonLeader.configSupplyCurrentLimit(current_limit);
    leftTalonFollower.configSupplyCurrentLimit(current_limit);
    rightTalonLeader.configSupplyCurrentLimit(current_limit);
    rightTalonFollower.configSupplyCurrentLimit(current_limit);

    // Set up the encoders
    leftEncoder = leftSparkMaxFollower.getEncoder();
    rightEncoder = rightSparkMaxFollower.getEncoder();

    // set the encorder speed conversion factor from native units to meters per
    // second.
    leftEncoder.setVelocityConversionFactor(kPositionConversionFactor);
    rightEncoder.setVelocityConversionFactor(kPositionConversionFactor);

    // set the encorder position conversion factor from native units to meters.
    leftEncoder.setPositionConversionFactor(kPositionConversionFactor);
    rightEncoder.setPositionConversionFactor(kPositionConversionFactor);

    // TODO: Remove accelerometer and navigation from the drivetrain
    // accelerometer = new BuiltInAccelerometer();

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
  }
}