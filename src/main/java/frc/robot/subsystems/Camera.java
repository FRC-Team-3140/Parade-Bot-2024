// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.pathfindToApriltag;
import frc.robot.commands.turnToFaceApriltag;
import frc.robot.subsystems.DifferentialDriveSubsystem;

public class Camera extends SubsystemBase {

  private static Camera instance = null;

  private NetworkTableInstance inst = NetworkTableInstance.getDefault();

  private PhotonCamera april = null;
  private PhotonCamera notes = null;

  private boolean connected = false;
  private int connectionAttempts = 2;

  // The heartbeat is a value in the Photonvision Networktable that continually
  // changes.
  private double heartbeat = 0;
  private double previousResult = -1;

  // This thread allows this connection check to run in the background and not
  // block other Subsystems.
  private Thread attemptReconnection = new Thread(this::attemptToReconnect);

  // Must not start at 0 so check doesn't run early
  private int count = 1;

  // Time to delay periodic method Networktable connection check. IN Mili-Seconds!!
  private double delayTime = 5000.0;

  // Time to delay connection attempts is in SECONDS! - TK
  private double attemptDelay;

  private DifferentialDriveSubsystem drivetrain;
  private Pose2d currentSwervePose2d;
  private double currentX;
  private double currentY;
  private double newX;
  private double newY;

  private double percentTravelDist = 0.8; // Must be < 1

  private int speakerAprilTag;
  private int ampAprilTag;
  private int sourceAprilTag;

  public class aprilTagLocation {
    public final boolean isDetected;
    public final double distance;
    public final double angle;
    public final int id;

    public aprilTagLocation(boolean isDetected, double dist, double angle, int id) {
      this.isDetected = isDetected;
      this.distance = dist;
      this.angle = angle;
      this.id = id;
    }
  }

  public class NoteLocation {
    public final boolean isDetected;
    public final double distance;
    public final double angle;
  
    public NoteLocation(boolean isDetected, double dist, double angle, int id) {
      this.isDetected = isDetected;
      this.distance = dist;
      this.angle = angle;
    }
  }

  private Camera(DifferentialDriveSubsystem drivetrain, int PhotonvisionConnectionAttempts, double delayBetweenAttempts) {
    attemptDelay = delayBetweenAttempts;

    while (connected == false && connectionAttempts <= PhotonvisionConnectionAttempts) {
      if (inst.getTable("photonvision").getSubTables().contains("april")) {
        connected = true;
        System.out.println("PhotonVision is connected and is probably working as expected...");
        break;
      } else {
        System.err.println("Photonvision Not Connected Properly!");
        connected = false;
        System.out.println("Attempt: " + connectionAttempts + "\nChecking for PhotonVision connection in "
            + attemptDelay + " seconds.");
        Timer.delay(attemptDelay);
        connectionAttempts++;
      }
    }

    if (connected == true) {
      aprilGetInstance();
      notesGetInstance();
    }

    drivetrain = drivetrain;

    configureTeam();
  }

  public static Camera getInstance() {
    if (instance == null) {
      instance = new Camera(DifferentialDriveSubsystem.getInstance(), 5, 1);
    }
    return instance;
  }

  // Attempt reconnection method attempts to reinstantiate cameras on
  // reconnection.
  // The "singleton" is just here to return the already created instance if there
  // is one - TK
  private PhotonCamera aprilGetInstance() {
    if (april == null) {
      april = new PhotonCamera(inst, "april");
    }
    return april;
  }

  // Attempt reconnection method attempts to reinstantiate cameras on
  // reconnection.
  // The "singleton" is just here to return the already created instance if there
  // is one - TK
  private PhotonCamera notesGetInstance() {
    if (notes == null) {
      notes = new PhotonCamera(inst, "notes");
    }
    return notes;
  }

  private void configureTeam() {
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red){
      speakerAprilTag = 4; // Apriltag on left
      ampAprilTag = 5;
      sourceAprilTag = 10; // Apriltag on left
    } else {
      speakerAprilTag = 8; // Apriltag on left
      ampAprilTag = 6;
      sourceAprilTag = 2; // Apriltag on left
    }
  }

  private boolean testConnection() {
    // Gets new result from april camera and test if it's equal to the previous
    // result
    if (heartbeat == previousResult) {
      connected = false;
      heartbeat = inst.getTable("photonvision").getSubTable("april").getEntry("heartbeat").getDouble(0);
    } else {
      connected = true;
    }

    return connected;
  }

  private void attemptToReconnect() {
    System.out.println(
        "!!!!!!!!!!!!!!!!!!!!\nPhotonvision is no longer connected properly.\nAttempting reconnection\n!!!!!!!!!!!!!!!!!!!!");

    while (connected == false) {
      if (testConnection() == true) {
        connected = true;
        System.out.println("PhotonVision is connected and is probably working as expected...");
        break;
      } else {
        System.err.println("Photonvision Not Connected Properly!");
        connected = false;
        System.out.println("Checking for PhotonVision connection in " + attemptDelay + " seconds.");
        Timer.delay(attemptDelay);
      }

      aprilGetInstance();
      notesGetInstance();
    }
    // System.out.println(heartbeat);
  }

  public boolean getStatus() {
    return connected;
  }

  @Override
  public void periodic() {
    previousResult = heartbeat;
    heartbeat = inst.getTable("photonvision").getSubTable("april").getEntry("heartbeat").getDouble(0);

    // Was using Timer.delay() function here, but this caused issues with the other
    // subsystems...
    // Dividing by ideal 20 Ms roboRio loop time. - TK
    if ((count % (delayTime/20)) == 0 && !attemptReconnection.isAlive() && testConnection() == false) {
      try {
        attemptReconnection.start();
      } catch (IllegalThreadStateException e) {
        System.out.println("Exception occured in Camera: \n" + e + "\nThread state: " + attemptReconnection.getState());
      }
    }

    count++;

    // inst.getTable("Vision").getSubTable("Camera").getEntry("X:
    // ").setDouble(getApriltagDistX(speakerAprilTag));
    // inst.getTable("Vision").getSubTable("Camera").getEntry("Y:
    // ").setDouble(getApriltagDistY(speakerAprilTag));
    // inst.getTable("Vision").getSubTable("Camera").getEntry("Degrees:
    // ").setDouble(getDegToApriltag(speakerAprilTag));

    aprilTagLocation tag = getAprilTagLocation(speakerAprilTag);
    inst.getTable("Vision").getSubTable("Camera").getEntry("ID: ").setInteger(tag.id);
    inst.getTable("Vision").getSubTable("Camera").getEntry("Detected: ").setBoolean(tag.isDetected);
    inst.getTable("Vision").getSubTable("Camera").getEntry("Dist: ").setDouble(tag.distance);
    inst.getTable("Vision").getSubTable("Camera").getEntry("Degrees: ").setDouble(tag.angle);
  }

  public int getApriltagID() {
    // If this function returns a 0, that means there is not any detected targets

    if (connected && april.getLatestResult().hasTargets()) {
      return april.getLatestResult().getBestTarget().getFiducialId();
    } else {
      return -1;
    }
  }

  public double getApriltagYaw() {
    // If this function returns a 999, that means there is not any detected targets

    if (connected && april.getLatestResult().hasTargets()) {
      return april.getLatestResult().getBestTarget().getYaw();
    } else {
      return 999;
    }
  }

  public double getApriltagYaw(int id) {
    // If this function returns a 999, that means there is not any detected targets

    if (connected && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target.getFiducialId() == id) {
          return target.getYaw();
        }
      }
      return 0;
    } else {
      return 999;
    }
  }

  public double getApriltagPitch() {
    // If this function returns a 999, that means there is not any detected targets

    if (connected && april.getLatestResult().hasTargets()) {
      return april.getLatestResult().getBestTarget().getPitch();
    } else {
      return 999;
    }
  }

  public double getApriltagDistX() {
    // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
    // phase.
    if (connected && april.getLatestResult().hasTargets()) {
      return april.getLatestResult().getBestTarget().getBestCameraToTarget().getY();
    } else {
      return 0;
    }
  }

  public double getApriltagDistX(int id) {
    // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
    // phase.
    if (connected && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target.getFiducialId() == id) {
          return target.getBestCameraToTarget().getY();
        }
      }
      return 0;
    } else {
      return 0;
    }
  }

  public double getApriltagDistY() {
    // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
    // phase.
    if (connected && april.getLatestResult().hasTargets()) {
      return april.getLatestResult().getBestTarget().getBestCameraToTarget().getX();
    } else {
      return 0;
    }
  }

  public double getApriltagDistY(int id) {
    // This coordinate is relative to the robot w/t the Photonvision axis 90* out of
    // phase.
    if (connected && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target.getFiducialId() == id) {
          return target.getBestCameraToTarget().getX();
        }
      }
      return 0;
    } else {
      return 0;
    }
  }

  public double getAprilTagDist() {
    double dist;

    dist = Math.sqrt((Math.pow(getApriltagDistX(), 2) + Math.pow(getApriltagDistY(), 2)));

    return dist;
  }

  public double getDegToApriltag() {
    // Usable range of values with best consistancy: -50 - 50 With respect to
    // camera. - TK
    if (connected && april.getLatestResult().hasTargets()) {
      // double targetYaw = getApriltagYaw();
      double requiredTurnDegrees;

      /*
       * Takes Photonvision Z angle theta value (3D processing mode on camera) and
       * gets sign,
       * if sign is negative (apriltag is on left of frame), it will turn left the #
       * of degs.
       * that arcTan or inverse tan returns from the X & Y coorinates. Else it turns
       * right
       * by the arcTan or inverse tan of the X & Y coordinates. - TK
       */

      // if (Math.signum(targetYaw) == -1) {
      // requiredTurnDegrees = -Math.toDegrees(Math.atan2(getApriltagDistY(),
      // getApriltagDistX()));
      // } else {
      // requiredTurnDegrees = Math.toDegrees(Math.atan2(getApriltagDistY(),
      // getApriltagDistX()));
      // }

      // Need to use the getX method that we wrote for Y in atan because it returns
      // the Photon Y. - TK
      requiredTurnDegrees = Math.toDegrees(Math.atan2(getApriltagDistX(), getApriltagDistY()));

      System.out.println(requiredTurnDegrees);

      return requiredTurnDegrees;
    } else {
      return 0;
    }
  }

  public double getDegToApriltag(int id) {
    // Usable range of values with best consistancy: -50 - 50 With respect to
    // camera. - TK
    if (connected && april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target.getFiducialId() == id) {
          double requiredTurnDegrees;

          /*
           * Takes Photonvision Z angle theta value (3D processing mode on camera) and
           * gets sign,
           * if sign is negative (apriltag is on left of frame), it will turn left the #
           * of degs.
           * that arcTan or inverse tan returns from the X & Y coorinates. Else it turns
           * right
           * by the arcTan or inverse tan of the X & Y coordinates. - TK
           */

          // Need to use the getX method that we wrote for Y in atan because it returns
          // the Photon Y. - TK
          requiredTurnDegrees = Math.toDegrees(Math.atan2(getApriltagDistX(id), getApriltagDistY(id)));

          return requiredTurnDegrees;
        }
      }
      return 0;
    } else {
      return 0;
    }
  }

  public aprilTagLocation getAprilTagLocation(int id) {
    if (april.getLatestResult().hasTargets()) {
      for (PhotonTrackedTarget target : april.getLatestResult().getTargets()) {
        if (target.getFiducialId() == id) {
          double dist = getApriltagDistY(id);
          double deg = getDegToApriltag(id);

          return new aprilTagLocation(true, dist, deg, id);
        }
      }
    }
    return new aprilTagLocation(false, 0, 0, -1);
  }

  public double getNoteDistance() {
    // If this function returns a 0, that means there is not any detected targets

    // Need to wait until cameras are on Final Robot because calculation requires
    // specific
    // measurements to the camera.

    notes.getLatestResult().getBestTarget();
    PhotonUtils.calculateDistanceToTargetMeters(0, 0, 0, 0);

    return 0.0;
  }

  public SequentialCommandGroup pathfindToAprilTag() {
    SequentialCommandGroup goToAprilTag = new SequentialCommandGroup(
      new turnToFaceApriltag(speakerAprilTag, drivetrain, Camera.getInstance()),
      new InstantCommand(() -> {
        currentSwervePose2d = drivetrain.getPose();
        currentX = currentSwervePose2d.getX();
        currentY = currentSwervePose2d.getY();
        newX = getApriltagDistX(speakerAprilTag);
        newY = percentTravelDist * getApriltagDistY(speakerAprilTag);
      }),
      new pathfindToApriltag(new Pose2d((currentX - newX), (currentY - newY), new Rotation2d(0)), Camera.getInstance(),
          drivetrain),
      new turnToFaceApriltag(speakerAprilTag, drivetrain, Camera.getInstance())
    );
    
    // Fallback code. NOT tested!!!!! - TK
    // SequentialCommandGroup goToAprilTag = new SequentialCommandGroup(
    //   // new InstantCommand(() -> swerveDrive.resetPose(new Pose2d(4.5, 6.5, new Rotation2d(Math.toRadians(swerveDrive.getPose().getRotation().getDegrees()))))),
    //   new InstantCommand(() -> System.out.println("X: " + swerveDrive.getPose().getX() + " Y: " + swerveDrive.getPose().getY())),
    //   // new Pathfinding(new Pose2d((swerveDrive.getPose().getX()/* + 0.25*/), (swerveDrive.getPose().getY()/* + 0.25*/), new Rotation2d(Math.toRadians(swerveDrive.getPose().getRotation().getDegrees() + 45))), Camera.getInstance(), swerveDrive),
    //   new InstantCommand(() -> System.out.println("DistX to Target: "+getApriltagDistX(2))),
    //   // new InstantCommand(() -> RobotContainer.m_robotDrive.drive(0, Math.signum(getApriltagDistX(2)), getDegToApriltag(2), false)),
    //   new InstantCommand(() -> RobotContainer.swerve.drive(0,1.0*Math.signum(getApriltagDistX(2)), 0, false)),
    //   new InstantCommand(() -> System.out.println("DONE"))
    // );

    return goToAprilTag;
  }

// public SequentialCommandGroup pathfindToAprilTag(int id) {
// return new SequentialCommandGroup(
// turnToFaceApriltag(id),
// new InstantCommand(() -> {
// currentSwervePose2d = swerveDrive.getPose();
// currentX = currentSwervePose2d.getX();
// currentY = currentSwervePose2d.getY();
// newX = getApriltagDistX(id);
// newY = getApriltagDistY(id);

// new pathfindToApriltag(new Pose2d((currentX - newX), (currentY - newY), new
// Rotation2d(0)), Camera.getInstance(),
// swerveDrive).schedule();
// }),
// turnToFaceApriltag(id));
// }
}