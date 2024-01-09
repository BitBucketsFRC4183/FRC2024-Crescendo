package org.bitbuckets;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.bitbuckets.drive.DriveSubsystem;
import xyz.auriium.mattlib2.Mattlib;

public class Robot extends TimedRobot {



    //our subsystems

    DriveSubsystem driveSubsystem;


    @Override
    public void robotInit() {
        CommandScheduler.getInstance().enable();
        Mattlib.LOOPER.runPreInit();
        Mattlib.LOOPER.runPostInit();

      /*  //Set up Drive
        SwerveModule[] modules = initSwerveModules();
        SwerveDriveKinematics kinematics = new SwerveDriveKinematics(SWERVE.translations());
        SimpleMotorFeedforward ff = new SimpleMotorFeedforward(SWERVE.ff_ks(), SWERVE.ff_kv(), SWERVE.ff_ka());
        this.driveSubsystem = new DriveSubsystem(modules,kinematics,ff);
*/
    }
/*

    public static SwerveModule[] initSwerveModules() {
        SwerveModule[] modules = new SwerveModule[4];
        for (int i = 0; i < modules.length; i++) {
            ILinearMotor driveMotor;
            IRotationalController steerController;
            IRotationEncoder absoluteEncoder;
            if (Robot.isSimulation()) {
                driveMotor = null;
                steerController = null;
                absoluteEncoder = null;
            } else {
                driveMotor = HardwareREV.linearSpark_noPID(DRIVES[i]);
                steerController = HardwareREV.rotationalSpark_builtInPID(STEERS[i], PIDS[i]);
                absoluteEncoder = new ThriftyEncoder();
            }
            modules[i] = new SwerveModule(driveMotor, steerController, absoluteEncoder);
        }
        return modules;
    }
*/

    @Override
    public void robotPeriodic() {
        Mattlib.LOOPER.runPeriodicLoop();

        CommandScheduler.getInstance().run();
    }
    
    
}
