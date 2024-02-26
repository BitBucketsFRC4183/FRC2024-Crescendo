package org.bitbuckets.vision;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.MathSharedStore;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.RobotContainer;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import xyz.auriium.mattlib2.loop.IMattlibHooked;
import xyz.auriium.yuukonstants.exception.ExplainedException;

import java.io.IOException;
import java.util.*;
import java.util.concurrent.atomic.AtomicReference;

import static org.bitbuckets.vision.VisionUtil.*;

public class VisionSubsystem  implements Subsystem, IMattlibHooked {

    final DoubleSubscriber xSub;
    final DoubleSubscriber ySub;
    final BooleanSubscriber detectedSub;

    final PhotonCamera camera_1;
    final PhotonCamera camera_2;
    final PhotonPoseEstimator estimator1;
    final PhotonPoseEstimator estimator2;
    final AprilTagDetector aprilTagDetector;

    public VisionPriority priority;
    Optional<PhotonTrackedTarget> lastTarget;
    Optional<PhotonTrackedTarget> bestTarget;

    PhotonPipelineResult cam1_result;
    PhotonPipelineResult cam2_result;

    static final String filepath = Filesystem.getDeployDirectory().getPath() + "/2024-crescendo.json";
    public static final AprilTagFieldLayout LAYOUT;
    static {
        try {
            LAYOUT = new AprilTagFieldLayout(filepath);
        } catch (IOException e) {
            throw new IllegalStateException(e.getMessage() + " here is why");
        }
    }

    public VisionSubsystem(PhotonCamera camera_1, PhotonCamera camera_2,
                           PhotonPoseEstimator estimator1, PhotonPoseEstimator estimator2,
                           AprilTagDetector aprilTagDetector) {

        this.camera_1 = camera_1;
        this.camera_2 = camera_2;
        this.estimator1 = estimator1;
        this.estimator2 = estimator2;
        this.aprilTagDetector = aprilTagDetector;
        this.priority = VisionPriority.SPEAKER;
        this.lastTarget = Optional.empty();
        this.bestTarget = Optional.empty();

        var table = NetworkTableInstance.getDefault().getTable("vision");

        this.xSub = table.getDoubleTopic("x").subscribe(0.0);
        this.ySub = table.getDoubleTopic("y").subscribe(0.0);
        this.detectedSub = table.getBooleanTopic("detected").subscribe(false);

        register();
        mattRegister();
    }

    @Override
    public void logPeriodic() {

    }

    private void logBT(PhotonTrackedTarget bt) {
        RobotContainer.VISION.log_best_target_name(lookingAt(bt.getFiducialId()).toString());
        RobotContainer.VISION.log_best_target_id(bt.getFiducialId());
        RobotContainer.VISION.log_best_target_pose(LAYOUT.getTagPose(bt.getFiducialId()).orElseThrow().toPose2d());
        RobotContainer.VISION.log_best_target_ambiguity(bt.getPoseAmbiguity());


        Pose2d bt_camera_to_tag = LAYOUT.getTagPose(bt.getFiducialId()).orElseThrow().
                plus(bt.getBestCameraToTarget()).toPose2d();

        RobotContainer.VISION.log_best_cameraToTag_pose(bt_camera_to_tag);

    }

    @Override
    public void periodic() {
        this.cam1_result = camera_1.getLatestResult();

        // disable camera result hehe
        this.cam2_result = new PhotonPipelineResult();


        Optional<PhotonTrackedTarget> optionalPhotonTrackedTarget = determineBestVisionTargetFromList(getAllTargets());
        if (optionalPhotonTrackedTarget.isPresent()) {
            PhotonTrackedTarget ptt = optionalPhotonTrackedTarget.get();


            logBT(ptt);
        }


        this.lastTarget = this.bestTarget;
        this.bestTarget = optionalPhotonTrackedTarget;
        this.lastTarget.ifPresent(lastTarget -> RobotContainer.VISION.log_last_target_name(lookingAt(lastTarget.getFiducialId()).toString()));

    }

    public enum VisionPriority {
        AMP,
        SPEAKER,
        NONE
    }


    @Override
    public ExplainedException[] verifyInit() {
        return new ExplainedException[0];
    }

    public PhotonTrackedTarget[] getAllTargets() {
        List<PhotonTrackedTarget> allTargets = new ArrayList<>();

        if (this.cam1_result.hasTargets()) {
            allTargets.addAll(this.cam1_result.getTargets());
        }

        if (this.cam2_result.hasTargets()) {
            allTargets.addAll(this.cam2_result.getTargets());
        }

        return allTargets.toArray(PhotonTrackedTarget[]::new);

    }




    Optional<PhotonTrackedTarget> determineBestVisionTargetFromList(PhotonTrackedTarget[] targets) {

        if (targets.length == 0) {
            return Optional.empty();
        }

        // default priorities, lower index represents high priority
        List<VisionFieldTarget> priorities = List.of(
                VisionFieldTarget.SPEAKER_CENTER,
                VisionFieldTarget.SPEAKER_LEFT,
                VisionFieldTarget.SPEAKER_RIGHT,
                VisionFieldTarget.AMP,
                VisionFieldTarget.SOURCE_LEFT,
                VisionFieldTarget.SOURCE_RIGHT,
                VisionFieldTarget.STAGE
        );

        PhotonTrackedTarget bestTarget = targets[0]; //bruh
        VisionFieldTarget bestTargetElement = lookingAt(bestTarget.getFiducialId());

        VisionUtil.TargetPriority targetPriority = VisionUtil.TargetPriority.AREA;

        // if priority enabled, BEST LOOKING AT TARGET SHOULD WIN
        // if priority enabled, not looking at ,best target wins
        for (PhotonTrackedTarget target : targets) {
            VisionFieldTarget targetElement = lookingAt(target.getFiducialId());

            if (this.priority == VisionPriority.SPEAKER) {
                List<VisionFieldTarget> speakers = VisionUtil.SPEAKERS;
                // tests if targetElement is one of these and if the bestTarget is not yet set to priority
                if ((speakers.contains(targetElement)) && !(speakers.contains(bestTargetElement))) {
                    bestTarget = target;
                    // tests if targetElement is one of these and if bestTarget is already set to priority
                } else if (speakers.contains(targetElement)) {
                    bestTarget = VisionUtil.compareTwoTargets(bestTarget, target, targetPriority);
                    // tests if targetElement is not one of these and if bestTarget is not set to priority
                } else if (!speakers.contains(bestTargetElement)) {
                    bestTarget = VisionUtil.compareTwoTargets(bestTarget, target, targetPriority);
                }

            } else if (this.priority == VisionPriority.AMP) {
                if ((targetElement == VisionFieldTarget.AMP) && (bestTargetElement != VisionFieldTarget.AMP)) {
                    bestTarget = target;
                } else if (targetElement == VisionFieldTarget.AMP) {
                    bestTarget = VisionUtil.compareTwoTargets(bestTarget, target, targetPriority);

                } else if (bestTargetElement != VisionFieldTarget.AMP) {
                    bestTarget = VisionUtil.compareTwoTargets(bestTarget, target, targetPriority);
                }


            } else if (this.priority == VisionPriority.NONE) {
                // if no operator priority selection, go by defaults
                if (priorities.indexOf(targetElement) < priorities.indexOf(bestTargetElement)) {
                    bestTarget = VisionUtil.compareTwoTargets(bestTarget,target,targetPriority);
                }
            }

            bestTargetElement = lookingAt(bestTarget.getFiducialId());
        }

        return Optional.of(bestTarget);
    }


    public Optional<PhotonTrackedTarget> getBestVisionTarget() {
        return getBestVisionTarget(false);
    }

    /**
     *
     * @param objectTargetLock When this is true the vision system will lock onto the same target as the last target if it can still see it
     * @return
     */
    public Optional<PhotonTrackedTarget> getBestVisionTarget(boolean objectTargetLock) {

        if (!objectTargetLock) return this.bestTarget;

        // techincally sets too
        // kind of fixed switching between speaker center / speaker side
        PhotonTrackedTarget bt = bestTarget.get();
        PhotonTrackedTarget lt = lastTarget.get();
        VisionFieldTarget bt_obj = lookingAt( bt.getFiducialId() );
        VisionFieldTarget lt_obj = lookingAt( lt.getFiducialId() );

        if ((SPEAKERS.contains(bt_obj) && SPEAKERS.contains(lt_obj) || (SOURCES.contains(bt_obj) && SOURCES.contains(lt_obj)))) {
            if (lt_obj == bt_obj) {
                return this.bestTarget;
            } else {
                PhotonTrackedTarget[] allTargets = getAllTargets();
                PhotonTrackedTarget[] prevObjectTargets = Arrays
                        .stream(allTargets)
                        .filter(trackedTarget -> lookingAt(trackedTarget.getFiducialId()).equals(lt_obj)).toArray(PhotonTrackedTarget[]::new);
                // if none of the current targets are of the samme type as the previous ones, just return the best target
                if (prevObjectTargets.length == 0) {
                    return Optional.empty();
                } else {
                    this.bestTarget = determineBestVisionTargetFromList(prevObjectTargets);
                    logBT(this.bestTarget.get());
                    return this.bestTarget;
                }
            }

        } else {
            return this.bestTarget;
        }
    }

    // estimated robot pose using cam 1
    public Optional<EstimatedRobotPose> estimateVisionRobotPose() {
        Optional<EstimatedRobotPose> optEstimatedRobotPose1 = estimator1.update(this.cam1_result);
        Optional<EstimatedRobotPose> optEstimatedRobotPose2 = estimator2.update(this.cam2_result);
        optEstimatedRobotPose1.ifPresent(esmPose1 -> RobotContainer.VISION.log_vision_robot_pose_1(esmPose1.estimatedPose.toPose2d()));
        optEstimatedRobotPose2.ifPresent(esmPose2 -> RobotContainer.VISION.log_vision_robot_pose_2(esmPose2.estimatedPose.toPose2d()));

        if (optEstimatedRobotPose1.isEmpty() && optEstimatedRobotPose2.isEmpty()) {
            return Optional.empty();
        } else if (optEstimatedRobotPose1.isEmpty()) {
            return optEstimatedRobotPose2;
        } else if (optEstimatedRobotPose2.isEmpty()) {
            return optEstimatedRobotPose1;
        } else {
            // weighting is done by least pose ambiguity
            // if multitag was used, then that weight is multiplied by own many targets and a constant (arbitrary weighting)
            EstimatedRobotPose estimatedRobotPose1 = optEstimatedRobotPose1.get();
            EstimatedRobotPose estimatedRobotPose2 = optEstimatedRobotPose2.get();
            double avgPoseAmbiguity1 = estimatedRobotPose1.targetsUsed.stream().mapToDouble(PhotonTrackedTarget::getPoseAmbiguity).sum() / estimatedRobotPose1.targetsUsed.size();
            double avgPoseAmbiguity2 = estimatedRobotPose2.targetsUsed.stream().mapToDouble(PhotonTrackedTarget::getPoseAmbiguity).sum() / estimatedRobotPose2.targetsUsed.size();

            RobotContainer.VISION.log_avgPoseAmbiguity1(avgPoseAmbiguity1);
            RobotContainer.VISION.log_avgPoseAmbiguity2(avgPoseAmbiguity2);

            // avgPoseAmbiguity should be between 0 and 1, less value has more weight
            double weight1 = 1 / (avgPoseAmbiguity1 / avgPoseAmbiguity2);
            double weight2 = 1;


            double multiTagWeightConstant = 1.5;
            if (estimatedRobotPose1.strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR ||
                    estimatedRobotPose1.strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_RIO) {
                weight1 *= (estimatedRobotPose1.targetsUsed.size() * multiTagWeightConstant);
            }

            if (estimatedRobotPose2.strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR ||
                    estimatedRobotPose2.strategy == PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_RIO) {
                weight2 *= (estimatedRobotPose2.targetsUsed.size() * multiTagWeightConstant);
            }

            RobotContainer.VISION.log_pose1_weight(weight1);
            RobotContainer.VISION.log_pose2_weight(weight1);
            Pose3d combinedWeightedPose = VisionUtil.combineTwoPoses(estimatedRobotPose1.estimatedPose, estimatedRobotPose2.estimatedPose, weight1, weight2);
            RobotContainer.VISION.log_combined_vision_robot_pose(combinedWeightedPose.toPose2d());

            List<PhotonTrackedTarget> allTargets = new ArrayList<>(estimatedRobotPose1.targetsUsed);
            allTargets.addAll(estimatedRobotPose2.targetsUsed);

            double timestamp = (estimatedRobotPose1.timestampSeconds*weight1 + estimatedRobotPose2.timestampSeconds*weight2) / (weight1 + weight2);

            PhotonPoseEstimator.PoseStrategy poseStrategy;
            if (weight1 >= weight2) { poseStrategy = estimatedRobotPose1.strategy; } else poseStrategy = estimatedRobotPose2.strategy;

            return Optional.of(new EstimatedRobotPose(combinedWeightedPose, timestamp, allTargets, poseStrategy));
        }
    }


    public Optional<Pose2d> getClosestNotePose() {
        return Optional.of(new Pose2d(7,4, new Rotation2d()));
}

    public PhotonCamera[] getCameras() {
        PhotonCamera[] cameras = new PhotonCamera[2];
        cameras[0] = camera_1;
        cameras[1] = camera_2;

        return cameras;
    }

}
