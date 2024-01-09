package org.bitbuckets.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import xyz.auriium.mattlib2.log.Conf;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.Log;

public interface DrivebaseComponent extends INetworkedComponent {

    @Conf("ks")
    double ff_ks();

    @Conf("kv")
    double ff_kv();

    @Conf("ka")
    double ff_ka();



    @Log("positions")
    void logSwervePositions(SwerveModulePosition[] positions);

    @Log("states")
    void logSwerveStates(SwerveModuleState[] states);

    @Log("pose2")
    void logPosition(Pose2d pose2d);

}
