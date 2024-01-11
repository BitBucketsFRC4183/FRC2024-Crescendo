package org.bitbuckets;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import org.bitbuckets.drive.DriveSubsystem;
import org.bitbuckets.drive.DrivebaseComponent;
import org.bitbuckets.vision.VisionComponent;
import xyz.auriium.mattlib2.Mattlib;
import xyz.auriium.mattlib2.MattlibSettings;

import static xyz.auriium.mattlib2.Mattlib.LOG;

public class Robot extends TimedRobot {




    //our subsystems

    DriveSubsystem driveSubsystem;


    @Override
    public void robotInit() {

        CommandScheduler.getInstance().enable();
        Mattlib.LOOPER.runPreInit();
        Mattlib.LOOPER.runPostInit();

        MattlibSettings.USE_LOGGING = true;

        VISION.x_position(2);



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



    //Components




    //Components MUST be created in the Robot class (because of how static bs works)
    public static final VisionComponent VISION = LOG.load(VisionComponent.class, "vision");

    public static final DrivebaseComponent DRIVE = LOG.load(DrivebaseComponent.class, "swerve");
    /*public static final MotorComponent[] DRIVES = MotorComponent.ofRange(
            LOG.load(CommonMotorComponent.class, "swerve/drive"),
            LOG.loadRange(IndividualMotorComponent.class, "swerve/drive", 4, Util.RENAMER)
    );
    public static final MotorComponent[] STEERS = MotorComponent.ofRange(
            LOG.load(CommonMotorComponent.class, "swerve/steer"),
            LOG.loadRange(IndividualMotorComponent.class, "swerve/steer", 4, Util.RENAMER)
    );
    public static final PIDComponent[] PIDS = PIDComponent.ofRange(
            LOG.load(CommonPIDComponent.class, "swerve/pid"),
            LOG.loadRange(IndividualPIDComponent.class, "swerve/pid", 4, Util.RENAMER)
    );*/


}
