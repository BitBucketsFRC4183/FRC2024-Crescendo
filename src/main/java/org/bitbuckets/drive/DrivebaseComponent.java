package org.bitbuckets.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import xyz.auriium.mattlib2.log.annote.Conf;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Log;

public interface DrivebaseComponent extends INetworkedComponent {

    @Conf("pigeonId") int pigeonCanId();
    @Conf("ks") double ff_ks();
    @Conf("kv") double ff_kv();
    @Conf("halfWidth") double halfWidth_meters();
    @Conf("halfBase") double halfBase_meters();

    @Log("positions") void logSwervePositions(SwerveModulePosition[] positions);
    @Log("states") void logSwerveStates(SwerveModuleState[] states);
    @Log("pose2") void logPosition(Pose2d pose2d);

}
