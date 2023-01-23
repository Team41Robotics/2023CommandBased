package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.HashMap;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class PhotonAprilTagCamerasSubsystem extends SubsystemBase{
    HashMap<Integer, Transform2d> taglocs = new HashMap<>();

     PhotonCamera[] cams;
    ShuffleboardTab camtab = Shuffleboard.getTab("Camera");

    static PhotonAprilTagCamerasSubsystem PhotonCamSubsystem;

    double last_time = 0;
    double ping = 0;
    Transform2d apriltags_pose = new Transform2d();
    boolean new_time = false;

    public void PhotonApriltagCameras(PhotonCamera[] cams) {
        taglocs.put(1, new Transform2d(new Translation2d(0, 0), new Rotation2d(0)));
        taglocs.put(2, new Transform2d(new Translation2d(0, -1.75), new Rotation2d(0)));

        this.cams = cams;

        camtab.addNumber("cam_last_timestamp", () -> last_time);
        camtab.addNumber("latency", () -> ping);
        camtab.addDouble("apriltags_x", () -> apriltags_pose.getX());
        camtab.addDouble("apriltags_y", () -> apriltags_pose.getY());
        camtab.addDouble("apriltags_theta", () -> apriltags_pose.getRotation().getDegrees());
    }

    public static PhotonAprilTagCamerasSubsystem getInstance(){
        if(PhotonCamSubsystem == null){
            PhotonCamSubsystem= new PhotonAprilTagCamerasSubsystem();
        }
        return PhotonCamSubsystem;
    }

}   