package org.bitbuckets.vision;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.RobotContainer;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonTrackedTarget;
import xyz.auriium.mattlib2.IPeriodicLooped;
import xyz.auriium.yuukonstants.exception.ExplainedException;

import java.util.List;
import java.util.Optional;

public class VisionSubsystem  implements Subsystem, IPeriodicLooped {

    final PhotonCamera camera_1;
    final PhotonCamera camera_2;
    public final AprilTagFieldLayout layout;
    final PhotonPoseEstimator estimator1;
    final PhotonPoseEstimator estimator2;
    final AprilTagDetector aprilTagDetector;
    final OperatorInput operatorInput;

    public VisionSubsystem(PhotonCamera camera_1, PhotonCamera camera_2, AprilTagFieldLayout layout,
                           PhotonPoseEstimator estimator1, PhotonPoseEstimator estimator2,
                           AprilTagDetector aprilTagDetector, OperatorInput operatorInput) {

        this.camera_1 = camera_1;
        this.camera_2 = camera_2;
        this.layout = layout;
        this.estimator1 = estimator1;
        this.estimator2 = estimator2;
        this.aprilTagDetector = aprilTagDetector;
        this.operatorInput = operatorInput;

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
                target = VisionFieldTarget.SPEAKER_RIGHT;
                break;
            case 8:
                target = VisionFieldTarget.SPEAKER_LEFT;
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

    // desired transform to move the robot to the desired final position, field relative pose
    public Optional<Transform3d> getDesiredTargetAlignTransform() {
        Optional<PhotonTrackedTarget> optTrackedTarget = getBestVisionTarget();
        ;
        if (optTrackedTarget.isEmpty()) {
            return Optional.empty();
        }

        PhotonTrackedTarget trackedTarget = optTrackedTarget.get();
        VisionFieldTarget target = lookingAt(trackedTarget.getFiducialId()).orElseThrow(); //field element
        RobotContainer.VISION.log_looking_at(target.toString());
        Transform3d tagTransform = getDesiredTransformFromTarget(target);
        Transform3d cameraToTagTransform = trackedTarget.getBestCameraToTarget();

        // subtract transforms for final robot transform
        Transform3d robotTransform = new Transform3d(
                cameraToTagTransform.getTranslation().minus(tagTransform.getTranslation()),
                cameraToTagTransform.getRotation().minus(tagTransform.getRotation())
        );
        RobotContainer.VISION.log_between_transformation(robotTransform.getTranslation().toTranslation2d().getDistance(new Translation2d(0, 0)));
        return Optional.of(robotTransform);

    }

    // Returns the transformation needed for final position depending on tag, apriltag relative
    public Transform3d getDesiredTransformFromTarget(VisionFieldTarget target) {

        // translations are in inches
        return switch (target) {
            case SPEAKER_CENTER ->
                    new Transform3d(new Translation3d(0d, 0d, Units.inchesToMeters(72)), new Rotation3d(0d, 0d, 0));
            case SPEAKER_LEFT ->
                    new Transform3d(new Translation3d(Units.inchesToMeters(24), 0d, Units.inchesToMeters(72)), new Rotation3d(0d, 0d, 0));
            case SPEAKER_RIGHT ->
                    new Transform3d(new Translation3d(Units.inchesToMeters(-24), 0d, Units.inchesToMeters(72)), new Rotation3d(0d, 0d, 0));
            case AMP ->
                    new Transform3d(new Translation3d(0d, 0d, Units.inchesToMeters(36)), new Rotation3d(0d, 0d, 0));
            case SOURCE_LEFT -> //relative to the robot
                    new Transform3d(new Translation3d(Units.inchesToMeters(20), 0d, Units.inchesToMeters(36)), new Rotation3d(0d, 0d, 0));
            case SOURCE_RIGHT -> //relative to the robot
                    new Transform3d(new Translation3d(Units.inchesToMeters(-20), 0d, Units.inchesToMeters(36)), new Rotation3d(0d, 0d, 0));
            case STAGE -> //relative to the robot
                    new Transform3d(new Translation3d(0d, 0d, Units.inchesToMeters(36)), new Rotation3d(0d, 0d, 0));
        };
    }

    // returns the vision target based on priorities
    public Optional<PhotonTrackedTarget> getBestVisionTarget() {
        Optional<PhotonTrackedTarget> vt1 = Optional.ofNullable(
                camera_1.getLatestResult().getBestTarget()
        );

        Optional<PhotonTrackedTarget> vt2 = Optional.ofNullable(
                camera_2.getLatestResult().getBestTarget()
        );

        // default priorities, lower index represents high priority
        List<VisionFieldTarget> priorities = List.of(
                VisionFieldTarget.STAGE,
                VisionFieldTarget.SPEAKER_CENTER,
                VisionFieldTarget.SPEAKER_LEFT,
                VisionFieldTarget.SPEAKER_RIGHT,
                VisionFieldTarget.AMP,
                VisionFieldTarget.SOURCE_LEFT,
                VisionFieldTarget.SOURCE_RIGHT
        );


        // Chooses best target based on priority list
        if (vt1.isEmpty()) {
            return vt2;
        }

        if (vt2.isEmpty()) {
            return vt1;
        }

        if (vt1.get().equals(vt2.get())) {
            // best target is the same tag, so return that tag
            return vt1;
        }

        // cameras see different tags, so choose based on priority
        Optional<VisionFieldTarget> vt1Element = lookingAt(vt1.get().getFiducialId());
        Optional<VisionFieldTarget> vt2Element = lookingAt(vt2.get().getFiducialId());

        // check if the operator has a priority currently enabled
        // if both priorities are enabled, by default it does the speaker first
        if (operatorInput.getSpeakerPriorityToggleState()) {
            if (vt1Element.get() == VisionFieldTarget.SPEAKER_CENTER ||
                    vt1Element.get() == VisionFieldTarget.SPEAKER_LEFT ||
                    vt1Element.get() == VisionFieldTarget.SPEAKER_RIGHT) {
                return vt1;
            } else if (vt2Element.get() == VisionFieldTarget.SPEAKER_CENTER ||
                    vt2Element.get() == VisionFieldTarget.SPEAKER_LEFT ||
                    vt2Element.get() == VisionFieldTarget.SPEAKER_RIGHT) {
                return vt2;
            }
        } else if (operatorInput.getAmpPriorityToggleState()) {
            if (vt1Element.get() == VisionFieldTarget.AMP) {
                return vt1;
            } else if (vt2Element.get() == VisionFieldTarget.AMP) {
                return vt2;
            }
        } else {
            // if no operator priority selection, go by defaults
            if (priorities.indexOf(vt1Element) < priorities.indexOf(vt2Element)) {
                return vt1;
            } else {
                return vt2;
            }
        }

        return Optional.empty();
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
