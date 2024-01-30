package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonVisionSubsystem extends SubsystemBase {
    private final PhotonCamera aprilTagCamera;
    private final PhotonCamera noteCamera;

    public PhotonVisionSubsystem(String aprilTagCameraName, String noteCameraName) {
        aprilTagCamera = new PhotonCamera(aprilTagCameraName);
        noteCamera = new PhotonCamera(noteCameraName);
    }

    public double getAprilTagDistance() {
        // TODO: Need to estimate distance from area
        return aprilTagCamera.getLatestResult().getBestTarget().getArea();
    }

    public double getAprilTagAngle() {
        return aprilTagCamera.getLatestResult().getBestTarget().getYaw();
    }

    public double getNoteDistance() {
        // TODO: Need to estimate distance from area
        return noteCamera.getLatestResult().getBestTarget().getArea();
    }

    public double getNoteAngle() {
        return noteCamera.getLatestResult().getBestTarget().getYaw();
    }

    public PhotonTrackedTarget getBestAprilTag() {
        return aprilTagCamera.getLatestResult().getBestTarget();
    }

    public PhotonTrackedTarget getBestNote() {
        return noteCamera.getLatestResult().getBestTarget();
    }
    
}