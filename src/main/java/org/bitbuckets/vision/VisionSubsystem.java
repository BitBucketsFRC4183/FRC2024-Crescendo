package org.bitbuckets.vision;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.networktables.*;
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

import java.util.*;
import java.util.concurrent.atomic.AtomicReference;

import static org.bitbuckets.vision.VisionUtil.*;

public class VisionSubsystem  implements Subsystem, IMattlibHooked {

    DoubleSubscriber xSub;
    DoubleSubscriber ySub;
    BooleanSubscriber detectedSub;

    // use an AtomicReference to make updating the value thread-safe
    final AtomicReference<Double> xValue = new AtomicReference<Double>();
    final AtomicReference<Double> yValue = new AtomicReference<Double>();
    final AtomicReference<Boolean> detectedValue = new AtomicReference<Boolean>();

    // retain listener handles for later removal
    int xValueListenerHandle;
    int yValueListenerHandle;
    int detectedValueListenerHandle;

    final PhotonCamera camera_1;
    final PhotonCamera camera_2;
    public final AprilTagFieldLayout layout;
    final PhotonPoseEstimator estimator1;
    final PhotonPoseEstimator estimator2;
    final AprilTagDetector aprilTagDetector;

    public VisionPriority priority;

    private Optional<PhotonTrackedTarget> lastTarget;
    private Optional<PhotonTrackedTarget> bestTarget;

    PhotonPipelineResult cam1_result;
    PhotonPipelineResult cam2_result;


    public VisionSubsystem(PhotonCamera camera_1, PhotonCamera camera_2, AprilTagFieldLayout layout,
                           PhotonPoseEstimator estimator1, PhotonPoseEstimator estimator2,
                           AprilTagDetector aprilTagDetector) {

        this.camera_1 = camera_1;
        this.camera_2 = camera_2;
        this.layout = layout;
        this.estimator1 = estimator1;
        this.estimator2 = estimator2;
        this.aprilTagDetector = aprilTagDetector;
        this.priority = VisionPriority.AMP;
        this.lastTarget = Optional.empty();
        this.bestTarget = Optional.empty();

        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        xSub = inst.getDefault().getDoubleTopic("vision/x").subscribe(0.0);
        ySub = inst.getDefault().getDoubleTopic("vision/y").subscribe(0.0);
        detectedSub = inst.getDefault().getBooleanTopic("vision/detected").subscribe(false);

        xValueListenerHandle = inst.addListener(
                xSub,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                    xValue.set(event.valueData.value.getDouble());
                });

        yValueListenerHandle = inst.addListener(
                ySub,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                    yValue.set(event.valueData.value.getDouble());
                });

        detectedValueListenerHandle = inst.addListener(
                detectedSub,
                EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                event -> {
                    detectedValue.set(event.valueData.value.getBoolean());
                });

        register();
        mattRegister();
    }

    private void logBT(PhotonTrackedTarget bt) {
        RobotContainer.VISION.log_best_target_name(lookingAt(bt.getFiducialId()).toString());
        RobotContainer.VISION.log_best_target_id(bt.getFiducialId());
        RobotContainer.VISION.log_best_target_pose(layout.getTagPose(bt.getFiducialId()).orElseThrow().toPose2d());
        RobotContainer.VISION.log_best_target_ambiguity(bt.getPoseAmbiguity());
    }
    @Override
    public void periodic() {
        this.cam1_result = camera_1.getLatestResult();
        this.cam2_result = camera_2.getLatestResult();

        Optional<PhotonTrackedTarget> optionalPhotonTrackedTarget = determineBestVisionTarget();
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
        // if (RobotContainer.DISABLER.vision_disabled()) return Optional.empty();
        //frc using 36h11 fam this year
        // aprilTagDetector.addFamily("36h11");
        return new ExplainedException[0];
    }

    public Optional<List<PhotonTrackedTarget>> getAllTargets() {
        List<PhotonTrackedTarget> allTargets = new ArrayList<>();

        if (this.cam1_result.hasTargets()) {
            allTargets.addAll(this.cam1_result.getTargets());
        }

        if (this.cam2_result.hasTargets()) {
            allTargets.addAll(this.cam2_result.getTargets());
        }

        if (allTargets.isEmpty()) {return Optional.empty();}
        return Optional.of(allTargets);

    }

    private Optional<PhotonTrackedTarget> determineBestVisionTarget() {
        Optional<List<PhotonTrackedTarget>> optAllTargets = getAllTargets();
        return optAllTargets.map(this::determineBestVisionTargetFromList);
    }
    private PhotonTrackedTarget determineBestVisionTargetFromList(List<PhotonTrackedTarget> targets) {

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

        PhotonTrackedTarget bestTarget = targets.get(0);
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

        return bestTarget;
    }


    public Optional<PhotonTrackedTarget> getBestVisionTarget() {
        return getBestVisionTarget(false);
    }

    public Optional<PhotonTrackedTarget> getBestVisionTarget(boolean objectTargetLock) {
        // techincally sets too
        // kind of fixed switching between speaker center / speaker side
        if (objectTargetLock && bestTarget.isPresent() && lastTarget.isPresent()) {
            PhotonTrackedTarget bt = bestTarget.get();
            PhotonTrackedTarget lt = lastTarget.get();
            VisionFieldTarget bt_obj = lookingAt(bt.getFiducialId());
            VisionFieldTarget lt_obj = lookingAt(lt.getFiducialId());

            if ((SPEAKERS.contains(bt_obj) && SPEAKERS.contains(lt_obj) || (SOURCES.contains(bt_obj) && SOURCES.contains(lt_obj)))) {
                if (lt_obj == bt_obj) {
                    return this.bestTarget;
                } else {
                    List<PhotonTrackedTarget> allTargets = getAllTargets().orElseThrow();
                    List<PhotonTrackedTarget> prevObjectTargets = allTargets.stream().filter(trackedTarget -> lookingAt(trackedTarget.getFiducialId()).equals(lt_obj)).toList();
                    // if none of the current targets are of the samme type as the previous ones, just return the best target
                    if (prevObjectTargets.isEmpty()) {
                        return Optional.empty();
                    } else {
                        this.bestTarget = Optional.of(determineBestVisionTargetFromList(prevObjectTargets));
                        logBT(this.bestTarget.get());
                        return this.bestTarget;
                    }
                }

            } else return this.bestTarget;

        } else return this.bestTarget;
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




    public synchronized void acceptSpaceData(double data) {

    }



    public double getNoteX() {
        double xPos = 0d;

        Double x = xValue.getAndSet(null);
        if (x != null) {
            xPos = x;
        }

        return xPos;
    }

    public double getNoteY() {
        double yPos = 0d;

        Double y = xValue.getAndSet(null);
        if (y != null) {
            yPos = y;
        }

        return yPos;
    }

    public boolean getNoteState() {
        boolean noteState = false;

        Boolean state = detectedValue.getAndSet(null);
        if (state != null) {
            noteState = state;
        }

        return noteState;
    }

    public PhotonCamera[] getCameras() {
        PhotonCamera[] cameras = new PhotonCamera[2];
        cameras[0] = camera_1;
        cameras[1] = camera_2;

        return cameras;
    }

}
