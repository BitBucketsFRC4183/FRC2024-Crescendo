package org.bitbuckets.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import xyz.auriium.mattlib2.log.annote.Conf;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Log;

public interface SwerveComponent extends INetworkedComponent {

    @Conf("pigeonId") int pigeonCanId();
    @Conf("ff_ks") double ff_ks();
    @Conf("ff_kv") double ff_kv();

    @Conf("magnitudeFWLim") double magnitudeFwLimit();
    @Conf("fieldOriented") boolean fieldOriented();

    //@Log("positions") void logSwervePositions(SwerveModulePosition[] positions);
    @Log("states") void logSwerveStates(SwerveModuleState[] states);
    @Log("desiredStates") void logDesiredStates(SwerveModuleState[] desiredStates);
    @Log("pose2") void logPosition(Pose2d pose2d);

}
