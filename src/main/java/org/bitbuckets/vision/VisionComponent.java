package org.bitbuckets.vision;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import xyz.auriium.mattlib2.log.annote.Conf;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Log;

import java.util.Optional;

public interface VisionComponent extends INetworkedComponent {

        @Log("best_target") void log_best_target_name(String target);
        @Log("best_target_id") void log_best_target_id(double id);
        @Log("best_target_ambiguity") void log_best_target_ambiguity(double ambiguity);
        @Log("best_target_pose") void log_best_target_pose(Pose2d pose);

        @Log("between_transformation_3d") void log_between_transformation(double distance);

        @Log("current_priority") void  log_current_priority(String priority);
      @Log("vision_robot_pose_1") void log_vision_robot_pose_1(Pose2d p1);
      @Log("vision_robot_pose_2") void log_vision_robot_pose_2(Pose2d p2);
      @Log("pose1Weight") void log_pose1_weight(double w);
        @Log("pose2Weight") void log_pose2_weight(double w);

        @Log("avgPoseAmbiguity1") void log_avgPoseAmbiguity1(double a);
    @Log("avgPoseAmbiguity2") void log_avgPoseAmbiguity2(double a);


      @Log("combined_vision_robot_pose") void log_combined_vision_robot_pose(Pose2d p);
      @Log("final_pose") void log_final_pose(Pose2d p);

}
