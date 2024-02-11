package org.bitbuckets.vision;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import xyz.auriium.mattlib2.log.annote.Conf;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Log;

import java.util.Optional;

public interface VisionComponent extends INetworkedComponent {

        @Log("best_target") void log_best_target(String target);
        @Log("best_target_id") void log_best_target_id(double id);
        @Log("between_transformation_3d") void log_between_transformation(double distance);

        @Log("current_priority") void  log_current_priority(String priority);
      @Log("vision_robot_pose_1") void log_vision_robot_pose_1(Pose2d p1);
      @Log("vision_robot_pose_2") void log_vision_robot_pose_2(Pose2d p2);

      @Log("combined_vision_robot_pose") void log_combined_vision_robot_pose(Pose2d p);

}
