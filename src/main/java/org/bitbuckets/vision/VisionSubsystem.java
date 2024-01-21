package org.bitbuckets.vision;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.Robot;
import org.bitbuckets.RobotContainer;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.OdometrySubsystem;
import org.bitbuckets.drive.SwerveModule;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import xyz.auriium.mattlib2.IPeriodicLooped;
import xyz.auriium.yuukonstants.exception.ExplainedException;

import java.util.Optional;
import java.util.function.Consumer;

public class VisionSubsystem  implements Subsystem, IPeriodicLooped {

    final PhotonCamera camera_1;
    final PhotonCamera camera_2;
    final AprilTagFieldLayout layout;
    final PhotonPoseEstimator estimator1;
    final PhotonPoseEstimator estimator2;
    final AprilTagDetector aprilTagDetector;

    final VisionSystemSim visionSystemSim;
    final OdometrySubsystem odometrySubsystem;
    public VisionSubsystem(OdometrySubsystem odometrySubsystem, PhotonCamera camera_1, PhotonCamera camera_2, AprilTagFieldLayout layout,
                           PhotonPoseEstimator estimator1, PhotonPoseEstimator estimator2,
                           AprilTagDetector aprilTagDetector) {

        this.odometrySubsystem = odometrySubsystem;
        this.camera_1 = camera_1;
        this.camera_2 = camera_2;
        this.layout = layout;
        this.estimator1 = estimator1;
        this.estimator2 = estimator2;
        this.aprilTagDetector = aprilTagDetector;



        // make separate vision sim interface because this shit is confusing if i have time

        if (Robot.isSimulation()) {
            this.visionSystemSim = new VisionSystemSim("main");
            visionSystemSim.addAprilTags(layout);

            SimCameraProperties cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(1280, 800, new Rotation2d(100));
            // Approximate detection noise with average and standard deviation error in pixels.
            cameraProp.setCalibError(0.25, 0.08);
            // Set the camera image capture frame rate (Note: this is limited by robot loop rate).
            cameraProp.setFPS(20);
            // The average and standard deviation in milliseconds of image data latency.
            cameraProp.setAvgLatencyMs(35);
            cameraProp.setLatencyStdDevMs(5);

            PhotonCameraSim camera1Sim = new PhotonCameraSim(camera_1, cameraProp);
            PhotonCameraSim camera2Sim = new PhotonCameraSim(camera_2, cameraProp);

            visionSystemSim.addCamera(camera1Sim,new Transform3d(0, 0, 0,
                                                new Rotation3d(0, 0, 0)));

            visionSystemSim.addCamera(camera2Sim,new Transform3d(0, 0, 0,
                    new Rotation3d(0, 0, 0)));

            camera1Sim.enableDrawWireframe(true);
            camera1Sim.enableProcessedStream(true);
            camera2Sim.enableDrawWireframe(true);
            camera2Sim.enableProcessedStream(true);


        } else this.visionSystemSim = null;


        register();
        mattRegister();
    }

    @Override
    public Optional<ExplainedException> verifyInit() {
        if (RobotContainer.DISABLER.vision_disabled()) return Optional.empty();
        //frc using 36h11 fam this year
        aprilTagDetector.addFamily("36h11");

        return Optional.empty();
    }

    @Override
    public void simulationPeriodic() {
        // visionSystemSim.update(odometrySubsystem.getCurrentPosition());
        visionSystemSim.update(new Pose2d(0, 0, new Rotation2d(0)));
        Field2d debugField = visionSystemSim.getDebugField();
        // debugField.getObject("EstimatedRobot").setPose(odometrySubsystem.getCurrentPosition());
        debugField.getObject("EstimatedRobot").setPose(new Pose2d(0, 0, new Rotation2d(0)));

//        Pose2d[] modulePoses = new Pose2d[swerveMods.length];
//        Pose2d swervePose = odometrySubsystem.getCurrentPosition();
//        for (int i = 0; i < swerveMods.length; i++) {
//            SwerveModule module = swerveMods[i];
//            SwerveModulePosition modPosition = module.getPosition();
//            modulePoses[i] = swervePose.transformBy(new Transform2d(
//                                            modPosition.distanceMeters, modPosition.angle));
//        }
//        return modulePoses;

        // debugField.getObject("EstimatedRobotModules").setPoses();


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

}
