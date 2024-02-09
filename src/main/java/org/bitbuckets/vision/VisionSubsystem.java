package org.bitbuckets.vision;

import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj2.command.Subsystem;
import org.bitbuckets.OperatorInput;
import org.bitbuckets.RobotContainer;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import xyz.auriium.mattlib2.IPeriodicLooped;
import xyz.auriium.yuukonstants.exception.ExplainedException;

import java.util.*;
import java.util.concurrent.atomic.AtomicReference;

import static org.bitbuckets.vision.VisionUtil.lookingAt;

public class VisionSubsystem  implements Subsystem, IPeriodicLooped {

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

    public VisionSubsystem(PhotonCamera camera_1, PhotonCamera camera_2, AprilTagFieldLayout layout,
                           PhotonPoseEstimator estimator1, PhotonPoseEstimator estimator2,
                           AprilTagDetector aprilTagDetector) {

        this.camera_1 = camera_1;
        this.camera_2 = camera_2;
        this.layout = layout;
        this.estimator1 = estimator1;
        this.estimator2 = estimator2;
        this.aprilTagDetector = aprilTagDetector;
        this.priority = VisionPriority.SPEAKER;

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

    public enum VisionPriority {
        AMP,
        SPEAKER,
        NONE
    }


    @Override
    public Optional<ExplainedException> verifyInit() {
        // if (RobotContainer.DISABLER.vision_disabled()) return Optional.empty();
        //frc using 36h11 fam this year
        // aprilTagDetector.addFamily("36h11");
        return Optional.empty();
    }

    // returns the vision target based on priorities
    public Optional<PhotonTrackedTarget> getBestVisionTarget() {
        // absolute priority
        PhotonPipelineResult result_cam1 = camera_1.getLatestResult();
        PhotonPipelineResult result_cam2 = camera_2.getLatestResult();

        List<PhotonTrackedTarget> allTargets = new ArrayList<>();

        if (result_cam1.hasTargets()) {
            allTargets.addAll(result_cam1.getTargets());
        }

        if (result_cam2.hasTargets()) {
            allTargets.addAll(result_cam2.getTargets());
        }

        if (allTargets.isEmpty()) {
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

        PhotonTrackedTarget bestTarget = allTargets.get(0);
        VisionFieldTarget bestTargetElement = lookingAt(bestTarget.getFiducialId());

        VisionUtil.TargetPriority targetPriority = VisionUtil.TargetPriority.AREA;

        // if priority enabled, BEST LOOKING AT TARGET SHOULD WIN
        // if priority enabled, not looking at ,best target wins
        for (PhotonTrackedTarget target : allTargets) {
            VisionFieldTarget targetElement = lookingAt(target.getFiducialId());

            if (this.priority == VisionPriority.SPEAKER) {
                List<VisionFieldTarget> speakers = Arrays.asList(VisionFieldTarget.SPEAKER_CENTER, VisionFieldTarget.SPEAKER_LEFT, VisionFieldTarget.SPEAKER_RIGHT);
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

    // estimated robot pose using cam 1
    public Optional<Pose3d> estimateVisionRobotPose_1() {
        return estimator1.update(camera_1.getLatestResult()).map(poseDat -> poseDat.estimatedPose);
    }

    // estimated robot pose using cam 2
    public Optional<Pose3d> estimateVisionRobotPose_2() {
        return estimator2.update(camera_2.getLatestResult()).map(poseDat -> poseDat.estimatedPose);
    }

    public synchronized void acceptSpaceData(double data) {

    }

    // combines estimated poses by averaging
    // this is not currently used as we just use the cameratotag transform from the camera that has our desired target
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
