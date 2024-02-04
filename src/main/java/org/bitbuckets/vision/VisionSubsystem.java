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

    public Optional<VisionFieldTarget> lookingAt(int fiducialID) {
        VisionFieldTarget target;
        switch(fiducialID) {
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
    public Optional<Pose3d> getDesiredTargetAlignPose() {
        Optional<PhotonTrackedTarget> optTrackedTarget = getBestVisionTarget_1();;

        if (optTrackedTarget.isPresent()) {
            int fidID = optTrackedTarget.get().getFiducialId();

            var target = lookingAt(fidID).orElseThrow();
            RobotContainer.VISION.log_looking_at(target.toString());

            Optional<Transform3d> optTranfrom = getDesiredTransformFromTarget(target);

            if (optTranform.isPresent()) {
                // may throw error somewhere

                Pose3d aprilTagPose = layout.getTagPose(fidID).orElseThrow();
                Transform3d desiredTransformation = optTranform.get();

                Pose3d desiredPose = aprilTagPose.plus(desiredTransformation);
                return Optional.ofNullable(desiredPose);
            }
        }


        // TODO combine two cameras (weighting if see more than two aptriltags) in different part of a code



        return Optional.empty();
    }

    // TODO clean up naming and actually refractor everything
    // Returns the transformation for each target for desired final position
    // e.g. stop close to amp, far away from speaker
    public Optional<Transform3d> getDesiredTransformFromTarget(VisionFieldTarget thing) {

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
    public Optional<PhotonTrackedTarget> getBestVisionTarget_1() {


        Optional<PhotonTrackedTarget> vt  = Optional.ofNullable(
                camera_1.getLatestResult().getBestTarget()
        );

        return vt;
    }

    public Optional<PhotonTrackedTarget> getBestVisionTarget_2() {


        Optional<PhotonTrackedTarget> vt  = Optional.ofNullable(
                camera_2.getLatestResult().getBestTarget()
        );


        return vt;
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
