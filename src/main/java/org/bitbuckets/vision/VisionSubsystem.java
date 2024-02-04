package org.bitbuckets.vision;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.RobotContainer;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import xyz.auriium.mattlib2.IPeriodicLooped;
import xyz.auriium.yuukonstants.exception.ExplainedException;

import java.util.Optional;

public class VisionSubsystem  implements Subsystem, IPeriodicLooped {

    final PhotonCamera camera_1;
    final PhotonCamera camera_2;
    public final AprilTagFieldLayout layout;
    final PhotonPoseEstimator estimator1;
    final PhotonPoseEstimator estimator2;
    final AprilTagDetector aprilTagDetector;

    public VisionSubsystem(PhotonCamera camera_1, PhotonCamera camera_2, AprilTagFieldLayout layout,
                           PhotonPoseEstimator estimator1, PhotonPoseEstimator estimator2,
                           AprilTagDetector aprilTagDetector) {

        this.camera_1 = camera_1;
        this.camera_2 = camera_2;
        this.layout = layout;
        this.estimator1 = estimator1;
        this.estimator2 = estimator2;
        this.aprilTagDetector = aprilTagDetector;

        register();
        mattRegister();
    }

    public VisionFieldTarget lookingAt() {
        return null; //TODO FINISH THIS
    }

    @Override
    public Optional<ExplainedException> verifyInit() {
        // if (RobotContainer.DISABLER.vision_disabled()) return Optional.empty();
        //frc using 36h11 fam this year
        // aprilTagDetector.addFamily("36h11");

        return Optional.empty();
    }


    public Optional<Pose3d> estimateAprilTagTargetPose() {
        //TODO BLEND THE DATA BETWEEN THE TWO CAMERAS.


        return Optional.empty();
    }

    // Returns the transformation for each target for desired final position
    // e.g. stop close to amp, far away from speaker
    public Optional<Transform3d> getTagTransformBasedOnThing(VisionFieldTarget thing) {

        // translations are in inches
        return switch (thing) {
            case SPEAKER_CENTER ->
                    Optional.of(new Transform3d(new Translation3d(0d, 0d, Units.inchesToMeters(72)), new Rotation3d(0d, 0d, 0)));
            case SPEAKER_SIDE_LEFT ->
                    Optional.of(new Transform3d(new Translation3d(Units.inchesToMeters(24), 0d, Units.inchesToMeters(72)), new Rotation3d(0d, 0d, 0)));
            case SPEAKER_SIDE_RIGHT ->
                    Optional.of(new Transform3d(new Translation3d(Units.inchesToMeters(-24), 0d, Units.inchesToMeters(72)), new Rotation3d(0d, 0d, 0)));
            case AMP ->
                    Optional.of(new Transform3d(new Translation3d(0d, 0d, Units.inchesToMeters(36)), new Rotation3d(0d, 0d, 0)));
            case SOURCE_LEFT -> //relative to the robot
                    Optional.of(new Transform3d(new Translation3d(Units.inchesToMeters(20), 0d, Units.inchesToMeters(36)), new Rotation3d(0d, 0d, 0)));
            case SOURCE_RIGHT -> //relative to the robot
                    Optional.of(new Transform3d(new Translation3d(Units.inchesToMeters(-20), 0d, Units.inchesToMeters(36)), new Rotation3d(0d, 0d, 0)));
            case STAGE -> //relative to the robot
                    Optional.of(new Transform3d(new Translation3d(0d, 0d, Units.inchesToMeters(36)), new Rotation3d(0d, 0d, 0)));
        };

    }


    //BASIC INFORMATION GATHERING FROM CAMERAS
    public Optional<Pose3d> estimateBestVisionTarget_1() {


        Optional<Pose3d> vt  = Optional.ofNullable(
                camera_1.getLatestResult().getBestTarget()
        ).flatMap(tgt -> layout.getTagPose(tgt.getFiducialId()));

        vt.ifPresent(RobotContainer.VISION::log_vision_target_1);

        return vt;
    }

    public Optional<Pose3d> estimateBestVisionTarget_2() {

        Optional<Pose3d> vt = Optional.ofNullable(
                camera_2.getLatestResult().getBestTarget()
        ).flatMap(tgt -> layout.getTagPose(tgt.getFiducialId()));

        vt.ifPresent(RobotContainer.VISION::log_vision_target_2);
        return vt;
    }


    public Optional<Pose3d> estimateRobotPose_1() {

        Optional<Pose3d> p = Optional.ofNullable(
                camera_1.getLatestResult().getBestTarget()
        ).flatMap(tgt -> layout.getTagPose(tgt.getFiducialId()));

        p.ifPresent(RobotContainer.VISION::log_vision_robot_pose_1);
        return p;
    }

    public Optional<Pose3d> estimateRobotPose_2() {

        Optional<Pose3d> p = Optional.ofNullable(
                camera_2.getLatestResult().getBestTarget()
        ).flatMap(tgt -> layout.getTagPose(tgt.getFiducialId()));

        p.ifPresent(RobotContainer.VISION::log_vision_robot_pose_2);
        return p;
    }

    public Optional<Pose3d> estimateVisionRobotPose_1() {
        return estimator1.update(camera_1.getLatestResult()).map(poseDat -> poseDat.estimatedPose);
    }

    public Optional<Pose3d> estimateVisionRobotPose_2() {
        return estimator2.update(camera_2.getLatestResult()).map(poseDat -> poseDat.estimatedPose);
    }

    public Optional<Pose3d> combineEstimatedPose(Optional<Pose3d> pose1, Optional<Pose3d> pose2){

        if (pose1.isEmpty() && pose2.isEmpty()){
            return Optional.empty();
        } else if (pose1.isEmpty()){
            return pose2;
        } else if (pose2.isEmpty()){
            return pose1;
        }

        Translation3d translation = new Translation3d(
                (pose1.get().getX() + pose2.get().getX()) / 2.0,
                (pose1.get().getY() + pose2.get().getY()) / 2.0,
                (pose1.get().getZ() + pose2.get().getZ()) / 2.0
        );
        Rotation3d rotation = new Rotation3d(
                (pose1.get().getRotation().getX() +  pose2.get().getRotation().getX()) / 2.0,
                (pose1.get().getRotation().getY() +  pose2.get().getRotation().getY()) / 2.0,
                (pose1.get().getRotation().getZ() +  pose2.get().getRotation().getZ()) / 2.0
        );
        Pose3d result = new Pose3d(translation, rotation);

        return Optional.of(result);
    }

    public PhotonCamera[] getCameras() {
        PhotonCamera[] cameras = new PhotonCamera[2];
        cameras[0] = camera_1;
        cameras[1] = camera_2;

        return cameras;
    }

}
