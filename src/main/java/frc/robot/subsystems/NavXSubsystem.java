package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * This class is a subsystem for the NavX sensor.
 * It provides methods to get the yaw, pitch, roll, and compass heading from the sensor.
 * It also publishes these values to a NetworkTable for external use.
 */
public class NavXSubsystem extends SubsystemBase {
    private static NavXSubsystem instance = null;
    private AHRS ahrs;
    private NetworkTable table;
    private NetworkTableEntry yawEntry;
    private NetworkTableEntry pitchEntry;
    private NetworkTableEntry rollEntry;
    private NetworkTableEntry localAccelXEntry;
    private NetworkTableEntry localAccelYEntry;
    private NetworkTableEntry localAccelZEntry;
    private NetworkTableEntry compassHeadingEntry;

    /**
     * Private constructor for the NavXSubsystem class.
     * This constructor initializes the AHRS (NavX) sensor and sets up the NetworkTable entries for the sensor values.
     * 
     * The following NetworkTable entries are created:
     * - Yaw: The current yaw angle of the robot, in degrees.
     * - Pitch: The current pitch angle of the robot, in degrees.
     * - Roll: The current roll angle of the robot, in degrees.
     * - LocalAccelX: The current acceleration of the robot along the X axis, in G-forces.
     * - LocalAccelY: The current acceleration of the robot along the Y axis, in G-forces.
     * - LocalAccelZ: The current acceleration of the robot along the Z axis, in G-forces.
     * - CompassHeading: The current compass heading of the robot, in degrees.
     */
    private NavXSubsystem() {
        // Initialize the AHRS (NavX) sensor
        ahrs = new AHRS();

        // Get the NetworkTable instance
        table = NetworkTableInstance.getDefault().getTable("NavX");

        // Get the NetworkTable entries for yaw, pitch, roll, and compass heading
        yawEntry = table.getEntry("Yaw");
        pitchEntry = table.getEntry("Pitch");
        rollEntry = table.getEntry("Roll");

        localAccelXEntry = table.getEntry("LocalAccelX");
        localAccelYEntry = table.getEntry("LocalAccelY");
        localAccelZEntry = table.getEntry("LocalAccelZ");

        compassHeadingEntry = table.getEntry("CompassHeading");
    }
    /**
     * Gets the singleton instance of the NavXSubsystem class.
     * If the singleton instance doesn't exist yet, it's created.
     * 
     * @return The singleton instance of the NavXSubsystem class.
     */
    public static NavXSubsystem getInstance() {
        if (instance == null) {
            instance = new NavXSubsystem();
        }
        return instance;
    }

    /**
     * Gets the current yaw angle of the robot as a Rotation2d object.
     * The yaw angle represents the robot's rotation around the Z axis.
     * 
     * @return The current yaw angle as a Rotation2d object.
     */
    public Rotation2d getYaw() {
        return Rotation2d.fromDegrees(ahrs.getYaw());
    }

    /**
     * Gets the current pitch angle of the robot in degrees.
     * The pitch angle represents the robot's rotation around the Y axis.
     * 
     * @return The current pitch angle in degrees.
     */
    public double getPitch() {
        return ahrs.getPitch();
    }

    /**
     * Gets the current roll angle of the robot in degrees.
     * The roll angle represents the robot's rotation around the X axis.
     * 
     * @return The current roll angle in degrees.
     */
    public double getRoll() {
        return ahrs.getRoll();
    }

    /**
     * Gets the current acceleration of the robot along the X axis in G-forces.
     * This is a local acceleration value, which means it's relative to the robot's coordinate system.
     * 
     * @return The current acceleration along the X axis in G-forces.
     */
    public double getLocalAccelX() {
        return ahrs.getRawAccelX();
    }

    /**
     * Gets the current acceleration of the robot along the Y axis in G-forces.
     * This is a local acceleration value, which means it's relative to the robot's coordinate system.
     * 
     * @return The current acceleration along the Y axis in G-forces.
     */
    public double getLocalAccelY() {
        return ahrs.getRawAccelY();
    }

    /**
     * Gets the current acceleration of the robot along the Z axis in G-forces.
     * This is a local acceleration value, which means it's relative to the robot's coordinate system.
     * 
     * @return The current acceleration along the Z axis in G-forces.
     */
    public double getLocalAccelZ() {
        return ahrs.getRawAccelZ();
    }

    /**
     * Gets the current compass heading of the robot in degrees.
     * The compass heading is a value between 0 and 360 degrees, representing the robot's orientation relative to magnetic north.
     * 
     * @return The current compass heading in degrees.
     */
    public double getCompassHeading() {
        return ahrs.getCompassHeading();
    }

    /**
     * This method is called once per scheduler run.
     * It updates the NetworkTable entries with the current sensor values.
     * 
     * The following values are published to the NetworkTable:
     * - Yaw: The current yaw angle of the robot, in degrees. This is obtained from the NavX sensor.
     * - Pitch: The current pitch angle of the robot, in degrees. This is obtained from the NavX sensor.
     * - Roll: The current roll angle of the robot, in degrees. This is obtained from the NavX sensor.
     * - LocalAccelX: The current acceleration of the robot along the X axis, in G-forces. This is obtained from the NavX sensor.
     * - LocalAccelY: The current acceleration of the robot along the Y axis, in G-forces. This is obtained from the NavX sensor.
     * - LocalAccelZ: The current acceleration of the robot along the Z axis, in G-forces. This is obtained from the NavX sensor.
     * - CompassHeading: The current compass heading of the robot, in degrees. This is obtained from the NavX sensor.
     */
    @Override
    public void periodic() {
        yawEntry.setDouble(getYaw().getDegrees());
        pitchEntry.setDouble(getPitch());
        rollEntry.setDouble(getRoll());

        localAccelXEntry.setDouble(getLocalAccelX());
        localAccelYEntry.setDouble(getLocalAccelY());
        localAccelZEntry.setDouble(getLocalAccelZ());

        compassHeadingEntry.setDouble(getCompassHeading());
    }
}