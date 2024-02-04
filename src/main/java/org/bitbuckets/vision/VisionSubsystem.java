package org.bitbuckets.vision;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.RobotContainer;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;
import xyz.auriium.mattlib2.IPeriodicLooped;
import xyz.auriium.yuukonstants.exception.ExplainedException;

import javax.swing.text.html.Option;
import javax.xml.crypto.dsig.Transform;
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

    // converts apriltag ID to an element of the target enum
    public Optional<VisionFieldTarget> lookingAt(int fiducialID) {
        VisionFieldTarget target;
        switch (fiducialID) {
            case 1:
            case 9:
                target = VisionFieldTarget.SOURCE_RIGHT;
                break;
            case 2:
            case 10:
                target = VisionFieldTarget.SOURCE_LEFT;
                break;
            case 3:
                target = VisionFieldTarget.SPEAKER_SIDE_RIGHT;
                break;
            case 8:
                target = VisionFieldTarget.SPEAKER_SIDE_LEFT;
                break;
            case 5:
            case 6:
                target = VisionFieldTarget.AMP;
                break;
            case 7:
            case 4:
                target = VisionFieldTarget.SPEAKER_CENTER;
                break;
            case 11:
            case 12:
            case 13:
            case 14:
            case 15:
            case 16:
                target = VisionFieldTarget.STAGE;
                break;
            default:
                target = null;
        }
        return Optional.ofNullable(target);
    }

    @Override
    public Optional<ExplainedException> verifyInit() {
        // if (RobotContainer.DISABLER.vision_disabled()) return Optional.empty();
        //frc using 36h11 fam this year
        // aprilTagDetector.addFamily("36h11");


        return Optional.empty();
    }


    // field relative pose
    // desired transform to move the robot to the desired final position
    public Optional<Transform3d> getDesiredTargetAlignTransform() {
        Optional<PhotonTrackedTarget> optTrackedTarget = getBestVisionTarget();
        ;

        if (optTrackedTarget.isPresent()) {
            PhotonTrackedTarget trackedTarget = optTrackedTarget.get();
            int fidID = trackedTarget.getFiducialId();

            VisionFieldTarget target = lookingAt(fidID).orElseThrow();
            RobotContainer.VISION.log_looking_at(target.toString());
            Optional<Transform3d> optTransform = getDesiredTransformFromTarget(target);

            if (optTransform.isPresent()) {
                // may throw error somewhere

                Transform3d cameraToTagTransform = trackedTarget.getBestCameraToTarget();
                Transform3d desiredTransformation = optTransform.get();

                Transform3d betweenTransformation = new Transform3d(
                        cameraToTagTransform.getTranslation().minus(desiredTransformation.getTranslation()),
                        cameraToTagTransform.getRotation().minus(desiredTransformation.getRotation())
                );
                RobotContainer.VISION.log_between_transformation(betweenTransformation.getTranslation().toTranslation2d().getDistance(new Translation2d(0, 0)));
                return Optional.ofNullable(betweenTransformation);
            }
        }


        // TODO combine two cameras (weighting if see more than two aptriltags) in different part of a code


        return Optional.empty();
    }

    // TODO clean up naming and actually refractor everything
    // Returns the transformation for each target for desired final position
    // e.g. stop close to amp, far away from speaker
    public Optional<Transform3d> getDesiredTransformFromTarget(VisionFieldTarget target) {

        // translations are in inches
        return switch (target) {
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
    public Optional<PhotonTrackedTarget> getBestVisionTarget() {
        Optional<PhotonTrackedTarget> vt1 = Optional.ofNullable(
                camera_1.getLatestResult().getBestTarget()
        );

        Optional<PhotonTrackedTarget> vt2 = Optional.ofNullable(
                camera_2.getLatestResult().getBestTarget()
        );

        // decision tree, lowest ambiguity prioritized
        // TODO WHEN SAME ID, COMBINE DATA TOGETHER
        // TODO CREATE HAS TARGETS BASED OFF OF TWO CAMERAS
        if (vt1.isPresent() && vt2.isPresent()) {
            if (vt1.get().equals(vt2.get())) {
                return vt1;
            } else if (vt1.get().getPoseAmbiguity() <= vt2.get().getPoseAmbiguity()) {
                return vt1;
            } else return vt2;

        } else if (vt1.isPresent()) {
            return vt1;
        } else return vt2;
    }


    // estimated robot pose using cam 1
    public Optional<Pose3d> estimateVisionRobotPose_1() {
        return estimator1.update(camera_1.getLatestResult()).map(poseDat -> poseDat.estimatedPose);
    }

    // estimated robot pose using cam 2
    public Optional<Pose3d> estimateVisionRobotPose_2() {
        return estimator2.update(camera_2.getLatestResult()).map(poseDat -> poseDat.estimatedPose);
    }

    // combines estimated poses by averaging
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
