package org.bitbuckets.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import xyz.auriium.mattlib2.log.annote.Conf;
import xyz.auriium.mattlib2.log.INetworkedComponent;
import xyz.auriium.mattlib2.log.annote.Log;
import xyz.auriium.mattlib2.log.annote.Tune;

public interface SwerveComponent extends INetworkedComponent {

    @Conf("pigeonId") int pigeonCanId();
    @Conf("magnitudeFWLim") double magnitudeFwLimit();

    @Tune("field_oriented") boolean fieldOriented();
    @Conf("offset_tuning_mode") boolean offsetTuningMode();
    @Conf("use_velocity_pid") boolean useVelocityPID();
    @Conf("theta_mode_pSeed") double thetaModePSeed();

    //@Log("positions") void logSwervePositions(SwerveModulePosition[] positions);
    @Log("hall_based_states") void logHallEncoderBasedStates(SwerveModuleState[] states);
    @Log("absolute_based_states") void logAbsoluteBasedStates(SwerveModuleState[] states);
    @Log("desired_states") void logDesiredStates(SwerveModuleState[] desiredStates);
    @Log("pose2") void logPosition(Pose2d pose2d);
    @Log("rot") void logGyroRotation(double rot);
    @Log("endpos") void logEndpoint(Pose2d pose);

}
