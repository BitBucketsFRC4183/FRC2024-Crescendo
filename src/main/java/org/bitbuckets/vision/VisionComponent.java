package org.bitbuckets.vision;


import edu.wpi.first.math.geometry.Pose3d;
import xyz.auriium.mattlib2.log.annote.Conf;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Log;

import java.util.Optional;

public interface VisionComponent extends INetworkedComponent {
    @Log("vision_target_1") void log_vision_target_1(Pose3d vt1);
    @Log("vision_target_2") void log_vision_target_2(Pose3d vt2);

}
