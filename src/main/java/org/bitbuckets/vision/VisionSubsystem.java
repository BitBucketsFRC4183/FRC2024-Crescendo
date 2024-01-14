package org.bitbuckets.vision;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import org.bitbuckets.Robot;
import org.bitbuckets.RobotContainer;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

import java.util.Optional;

public class VisionSubsystem  {

    final PhotonCamera camera_1;
    final PhotonCamera camera_2;
    final AprilTagFieldLayout layout;
    final PhotonPoseEstimator estimator;
    final AprilTagDetector aprilTagDetector;

    public VisionSubsystem(PhotonCamera camera_1, PhotonCamera camera_2, AprilTagFieldLayout layout, PhotonPoseEstimator estimator, AprilTagDetector aprilTagDetector) {
        this.camera_1 = camera_1;
        this.camera_2 = camera_2;
        this.layout = layout;
        this.estimator = estimator;
        this.aprilTagDetector = aprilTagDetector;
    }

    public void init() {
        //example logging DELETE
        int id = RobotContainer.VISION.id();
        int result = id*4;
        RobotContainer.VISION.x_position(result);
        //frc using 36h11 fam this year
        aprilTagDetector.addFamily("36h11");
    }


    //BASIC INFORMATION GATHERING FROM CAMERAS
    public Optional<Pose3d> estimateBestVisionTarget_1() {

        return Optional.ofNullable(
                camera_1.getLatestResult().getBestTarget()
        ).flatMap(tgt -> layout.getTagPose(tgt.getFiducialId()));

    }

    public Optional<Pose3d> estimateBestVisionTarget_2() {

        return Optional.ofNullable(
                camera_2.getLatestResult().getBestTarget()
        ).flatMap(tgt -> layout.getTagPose(tgt.getFiducialId()));

    }

    public Optional<Pose3d> estimateVisionRobotPose_1() {
        return estimator.update(camera_1.getLatestResult()).map(poseDat -> poseDat.estimatedPose);
    }

    public Optional<Pose3d> estimateVisionRobotPose_2() {
        return estimator.update(camera_2.getLatestResult()).map(poseDat -> poseDat.estimatedPose);
    }




}
