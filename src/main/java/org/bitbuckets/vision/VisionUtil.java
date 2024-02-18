package org.bitbuckets.vision;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.Arrays;
import java.util.List;

public class VisionUtil {
    static List<VisionFieldTarget> SPEAKERS = Arrays.asList(VisionFieldTarget.SPEAKER_CENTER, VisionFieldTarget.SPEAKER_LEFT, VisionFieldTarget.SPEAKER_RIGHT);
    static List<VisionFieldTarget> SOURCES = Arrays.asList(VisionFieldTarget.SOURCE_LEFT, VisionFieldTarget.SOURCE_RIGHT);

    // converts apriltag ID to an element of the target enum
    public static VisionFieldTarget lookingAt(int fiducialID) {
        return switch (fiducialID) {
            case 1, 9 -> VisionFieldTarget.SOURCE_RIGHT;
            case 2, 10 -> VisionFieldTarget.SOURCE_LEFT;
            case 3 -> VisionFieldTarget.SPEAKER_RIGHT;
            case 8 -> VisionFieldTarget.SPEAKER_LEFT;
            case 5, 6 -> VisionFieldTarget.AMP;
            case 7, 4 -> VisionFieldTarget.SPEAKER_CENTER;
            case 11, 12, 13, 14, 15, 16 -> VisionFieldTarget.STAGE;
            default -> throw new IllegalStateException("Unexpected value: " + fiducialID);
        };
    }

    // Returns the transformation needed for final position depending on tag, apriltag relative
    public static Transform3d getDesiredTargetAlignTransform(PhotonTrackedTarget trackedTarget) {

        VisionFieldTarget target = lookingAt(trackedTarget.getFiducialId()); //field element
        Transform3d tagTransform = lookupRobotTransformFromTarget(target);


        Transform3d cameraToTagTransform = trackedTarget.getBestCameraToTarget();

        // subtract transforms for final robot transform
        Transform3d robotTransform = new Transform3d(
                cameraToTagTransform.getTranslation().minus(tagTransform.getTranslation()), cameraToTagTransform.getRotation().minus(tagTransform.getRotation().rotateBy(new Rotation3d(0, 0, 0))));

        return robotTransform;
    }


    public static Pose3d getDesiredTargetAlignPose(PhotonTrackedTarget trackedTarget) {
        VisionFieldTarget target = lookingAt(trackedTarget.getFiducialId()); //field element
        Transform3d tagTransform = lookupRobotTransformFromTarget(target);

        return VisionSubsystem.LAYOUT.getTagPose(trackedTarget.getFiducialId()).orElseThrow().plus(tagTransform);
    }

    // desired transform to move the robot to the desired final position, field relative pose, could be improved with mattlib
    public static Transform3d lookupRobotTransformFromTarget(VisionFieldTarget target) {
        // translations are in inches
        return switch (target) {
            case SPEAKER_CENTER ->
                    new Transform3d(Units.inchesToMeters(50), 0d, 0d, new Rotation3d(0d, 0d, 0d));
            case SPEAKER_LEFT ->
                    new Transform3d(new Translation3d(Units.inchesToMeters(24), 0d, Units.inchesToMeters(80)), new Rotation3d(0d, 0d, 0));
            case SPEAKER_RIGHT ->
                    new Transform3d(new Translation3d(Units.inchesToMeters(-24), 0d, Units.inchesToMeters(80)), new Rotation3d(0d, 0d, 0));
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

    public enum TargetPriority {
        AREA,
        POSE_AMBIGUITY
    }
    public static PhotonTrackedTarget compareTwoTargets(PhotonTrackedTarget t1, PhotonTrackedTarget t2, TargetPriority priorityType) {
        if (priorityType == TargetPriority.AREA) {
            if (t1.getArea() > t2.getArea()) {
                return t1;
            } else return t2;
        } else if (priorityType == TargetPriority.POSE_AMBIGUITY) {
            if (t1.getPoseAmbiguity() < t2.getPoseAmbiguity()) {
                return t1;
            } else return t2;
        } else return t1;
    }

    // combines estimated poses by averaging
    public static Pose3d combineTwoPoses(Pose3d pose1, Pose3d pose2, double poseWeight1, double poseWeight2) {
        Translation3d translation = new Translation3d(
                (pose1.getX()*poseWeight1 + pose2.getX()*poseWeight2) / (poseWeight1 + poseWeight2),
                (pose1.getY()*poseWeight1 + pose2.getY()*poseWeight2) / (poseWeight1 + poseWeight2),
                (pose1.getZ()*poseWeight1 + pose2.getZ()*poseWeight2) / (poseWeight1 + poseWeight2)
        );
        Rotation3d rotation = new Rotation3d(
                (pose1.getRotation().getX()*poseWeight1 + pose2.getRotation().getX()*poseWeight2) / (poseWeight1 + poseWeight2),
                (pose1.getRotation().getY()*poseWeight1 + pose2.getRotation().getY()*poseWeight2) / (poseWeight1 + poseWeight2),
                (pose1.getRotation().getZ()*poseWeight1 + pose2.getRotation().getZ()*poseWeight2) / (poseWeight1 + poseWeight2)
        );

        return new Pose3d(translation, rotation);
    }
}
