package org.bitbuckets;

import com.choreo.lib.ChoreoTrajectory;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.bitbuckets.climber.ClimberComponent;
import org.bitbuckets.climber.ClimberSubsystem;
import org.bitbuckets.commands.climber.MoveClimberCommand;
import org.bitbuckets.commands.drive.AugmentedDriveCommand;
import org.bitbuckets.commands.drive.AwaitThetaCommand;
import org.bitbuckets.commands.drive.MoveToAlignCommand;
import org.bitbuckets.commands.drive.traj.FollowTrajectoryExactCommand;
import org.bitbuckets.commands.groundIntake.FeedGroundIntakeGroup;
import org.bitbuckets.commands.groundIntake.GroundOuttakeCommand;
import org.bitbuckets.commands.shooter.*;
import org.bitbuckets.disabled.DisablerComponent;
import org.bitbuckets.disabled.KinematicGyro;
import org.bitbuckets.drive.*;
import org.bitbuckets.groundIntake.GroundIntakeComponent;
import org.bitbuckets.groundIntake.GroundIntakeSubsystem;
import org.bitbuckets.noteManagement.NoteManagementComponent;
import org.bitbuckets.noteManagement.NoteManagementSubsystem;
import org.bitbuckets.shooter.ShooterComponent;
import org.bitbuckets.shooter.ShooterSubsystem;
import org.bitbuckets.util.*;
import org.bitbuckets.vision.CamerasComponent;
import org.bitbuckets.vision.VisionComponent;
import org.bitbuckets.vision.VisionSimContainer;
import org.bitbuckets.vision.VisionSubsystem;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import xyz.auriium.mattlib.ctre.HardwareCTRE;
import xyz.auriium.mattlib2.MattConsole;
import xyz.auriium.mattlib2.Mattlib;
import xyz.auriium.mattlib2.MattlibSettings;
import xyz.auriium.mattlib2.auto.ff.GenerateFFComponent;
import xyz.auriium.mattlib2.hardware.*;
import xyz.auriium.mattlib2.hardware.config.*;
import xyz.auriium.mattlib2.log.ConsoleComponent;
import xyz.auriium.mattlib2.rev.HardwareREV;
import xyz.auriium.mattlib2.sim.HardwareSIM;
import xyz.auriium.mattlib2.utils.MockingUtil;
import xyz.auriium.yuukonstants.exception.ExplainedException;

import java.io.IOException;

import static xyz.auriium.mattlib2.Mattlib.LOG;

public class RobotContainer {


    public final DriveSubsystem driveSubsystem;
    public final OperatorInput operatorInput;
    public final ShooterSubsystem shooterSubsystem;
    public final OdometrySubsystem odometrySubsystem;
    public final VisionSubsystem visionSubsystem;
    public final VisionSimContainer visionSimContainer;
    public final ClimberSubsystem climberSubsystem;
    public final GroundIntakeSubsystem groundIntakeSubsystem;
    public final NoteManagementSubsystem noteManagementSubsystem;
    public final SwerveDriveKinematics kinematics;

    public final SendableChooser<Command> chooser;


    public PIDController xController;
    public PIDController yController;
    public ProfiledPIDController thetaController;

    public final MattConsole mainConsole;

    public RobotContainer() {

        //DO SETTINGS BEFORE PRE INIT
        MattlibSettings.USE_LOGGING = true;
        MattlibSettings.ROBOT = WhichRobotUtil.loadRobot();

        //THIS HAS TO RUN FIRST
        Mattlib.LOOPER.runPreInit();
        CommandScheduler.getInstance().enable();

        // load order matters!!!!!
        this.operatorInput = new OperatorInput();
        this.kinematics = loadKinematics();
        this.driveSubsystem = loadDriveSubsystem();
        this.visionSubsystem = loadVisionSubsystem();
        this.odometrySubsystem = loadOdometrySubsystem();
        this.shooterSubsystem = loadShooterSubsystem();
        this.climberSubsystem = loadClimberSubsystem();
        this.groundIntakeSubsystem = loadGroundIntakeSubsystem();
        this.noteManagementSubsystem = loadNoteManagementSubsystem();

        if (!DISABLER.vision_disabled() && Robot.isSimulation()) {
            PhotonCamera[] cameras = visionSubsystem.getCameras();
            this.visionSimContainer = new VisionSimContainer(
                    visionSubsystem,
                    odometrySubsystem,
                    cameras[0],
                    cameras[1],
                    visionSubsystem.layout
            );
        } else this.visionSimContainer = null;

        loadCommands();
        chooser = loadAutonomous();
        mainConsole = new MattConsole(CONSOLE);

        //THIS HAS TO RUN AT THE END
        Mattlib.LOOPER.runPostInit();
        var ee = Mattlib.LOOPER.runPostInit();

        for (ExplainedException e : ee) {
            throw e;
        }

        mainConsole.reportExceptions(ee);


        // disable the annoying driverstation joystick warning
        DriverStation.silenceJoystickConnectionWarning(true);
    }


    public void robotPeriodic() {

    }

    public void simulationPeriodic() {
        if (DISABLER.vision_disabled()) return;
        this.visionSimContainer.simulationPeriodic();
    }


    public void autonomousInit() {
        operatorInput.actuallyIsTeleop = false;

        chooser.getSelected().schedule();
    }

    public void disabledInit() {
        operatorInput.actuallyIsTeleop = false;
    }

    public void teleopInit() {
        operatorInput.actuallyIsTeleop = true;
    }

    public void testInit() {

        new AwaitThetaCommand(driveSubsystem, odometrySubsystem, thetaController, DRIVE_T_PID, Rotation2d.fromDegrees(90).getRadians())
                .andThen(Commands.runOnce(() -> {System.out.println("WE ARE DONE");})).schedule();

        //LinearFFGenRoutine groundTopFFRoutine = new LinearFFGenRoutine(TOP_GROUND_FFGEN, groundIntakeSubsystem.topMotor, groundIntakeSubsystem.topMotor);
        //LinearFFGenRoutine groundBottomFFRoutine = new LinearFFGenRoutine(BOTTOM_GROUND_FFGEN, groundIntakeSubsystem.bottomMotor, groundIntakeSubsystem.bottomMotor);
        //CTowerCommands.wrapRoutine(groundTopFFRoutine).schedule();
        //CTowerCommands.wrapRoutine(groundBottomFFRoutine).schedule();
      /*  RotationFFGenRoutine shooterFFRoutine = new RotationFFGenRoutine(SHOOTER_WHEEL_1_FFGEN, shooterSubsystem.leftMotor, shooterSubsystem.leftMotor );
        CTowerCommands.wrapRoutine(shooterFFRoutine).schedule();
        shooterFFRoutine = new RotationFFGenRoutine(SHOOTER_WHEEL_2_FFGEN, shooterSubsystem.rightMotor, shooterSubsystem.rightMotor);
        CTowerCommands.wrapRoutine(shooterFFRoutine).schedule();
*/
//        Command[] commands = new Command[4];
//        for (int i = 0; i < 4; i++) {
//            var motorWeAreTesting = driveSubsystem.modules[i].driveMotor;
//            LinearFFGenRoutine driveFFRoutine = new LinearFFGenRoutine(DRIVE_MOTORS_FFGEN[i],motorWeAreTesting, motorWeAreTesting);
//            commands[i]= CTowerCommands.wrapRoutine(driveFFRoutine);
//        }
//
//        new ParallelCommandGroup(commands).schedule();
    }

    public Command followTrajectory(String routine, String name, boolean isNeo) {
        String toAppend = isNeo ? "-neo" : "";

        ChoreoTrajectory trajectory = TrajLoadingUtil.getTrajectory(routine + toAppend, name + toAppend);
        return new FollowTrajectoryExactCommand(
                trajectory,
                odometrySubsystem,
                driveSubsystem,
                xController,
                yController,
                thetaController,
                false
        ).andThen(Commands.runOnce(driveSubsystem::commandWheelsToZero));
    }
    public Command followFirstTrajectory(String routine, String name, boolean isNeo) {
        String toAppend = isNeo ? "-neo" : "";

        ChoreoTrajectory trajectory = TrajLoadingUtil.getTrajectory(routine, name + toAppend);
        return new FollowTrajectoryExactCommand(
                trajectory,
                odometrySubsystem,
                driveSubsystem,
                xController,
                yController,
                thetaController,
                true
        );
    }


    SendableChooser<Command> loadAutonomous() {
        xController = new PIDController(DRIVE_X_PID.pConstant(), DRIVE_X_PID.iConstant(), DRIVE_X_PID.dConstant());
        yController = new PIDController(DRIVE_Y_PID.pConstant(), DRIVE_Y_PID.iConstant(), DRIVE_Y_PID.dConstant());
        thetaController = new ProfiledPIDController(
                DRIVE_T_PID.pConstant(),
                DRIVE_T_PID.iConstant(),
                DRIVE_T_PID.dConstant(),
                new TrapezoidProfile.Constraints(3, 3)
        );

        //TODO this is laggy

        boolean isNeo = MattlibSettings.ROBOT == MattlibSettings.Robot.MCR;

        var oneNoteCollect = new SequentialCommandGroup(
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem),
                followFirstTrajectory("oneNoteCollect", "oneNoteCollect-1", isNeo),
                new FinishGroundIntakeCommand(noteManagementSubsystem, groundIntakeSubsystem)
        );

        var twoNote = new SequentialCommandGroup(
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem),
                followFirstTrajectory("twoNote", "twoNote-1", isNeo),
                new FinishGroundIntakeCommand(noteManagementSubsystem, groundIntakeSubsystem),
                followTrajectory("twoNote", "twoNote-2", isNeo),
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem)
        );

        var shootLeave = new SequentialCommandGroup(
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem),
                followFirstTrajectory("shootLeave", "shootLeave", isNeo)
        );

        var twoNoteCollect = new SequentialCommandGroup(
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem),
                followFirstTrajectory("twoNoteCollect", "twoNoteCollect-1", isNeo),
                new FinishGroundIntakeCommand(noteManagementSubsystem, groundIntakeSubsystem),
                followTrajectory("twoNoteCollect", "twoNoteCollect-2", isNeo),
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem),
                followTrajectory("twoNoteCollect", "twoNoteCollect-3", isNeo),
                new FinishGroundIntakeCommand(noteManagementSubsystem, groundIntakeSubsystem)
        );

        var threeNote = new SequentialCommandGroup(
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem),
                followFirstTrajectory("threeNote", "threeNote-1", isNeo),
                new FinishGroundIntakeCommand(noteManagementSubsystem, groundIntakeSubsystem),
                followTrajectory("threeNote", "threeNote-2", isNeo),
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem),
                followTrajectory("threeNote", "threeNote-3", isNeo),
                new FinishGroundIntakeCommand(noteManagementSubsystem, groundIntakeSubsystem),
                followTrajectory("threeNote", "threeNote-4", isNeo),
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem)
        );

        var fourNote = new SequentialCommandGroup(
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem),
                followFirstTrajectory("fourNote", "fourNote-1", isNeo),
                new FinishGroundIntakeCommand(noteManagementSubsystem, groundIntakeSubsystem),
                followTrajectory("fourNote", "fourNote-2", isNeo),
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem),
                followTrajectory("fourNote", "fourNote-3", isNeo),
                new FinishGroundIntakeCommand(noteManagementSubsystem, groundIntakeSubsystem),
                followTrajectory("fourNote", "fourNote-4", isNeo),
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem),
                followTrajectory("fourNote", "fourNote-5", isNeo),
                new FinishGroundIntakeCommand(noteManagementSubsystem, groundIntakeSubsystem),
                followTrajectory("fourNote", "fourNote-6", isNeo),
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem)
        );

        var mvpTaxi = new SequentialCommandGroup(
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem),
                followFirstTrajectory("mvpTaxi", "mvpTaxi", isNeo)
        );

        //this is if drive isn't working for some reason and we just need to shoot during auto
        var shootOnly = new SequentialCommandGroup(
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem)
        );

        var rotationTest = new SequentialCommandGroup(
                followFirstTrajectory("rotation", "rotation-1", isNeo),
                followFirstTrajectory("rotation", "rotation-2", isNeo)
        );

        var twoNoteContested = new SequentialCommandGroup(
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem),
                followFirstTrajectory("twoNoteContested", "twoNoteContested-1", isNeo),
                new FinishGroundIntakeCommand(noteManagementSubsystem, groundIntakeSubsystem),
                followFirstTrajectory("twoNoteContested", "twoNoteContested-2", isNeo),
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem)
        );

        var twoNoteContestedAlt = new SequentialCommandGroup(
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem),
                followFirstTrajectory("twoNoteContestedAlt", "twoNoteContestedAlt-1", isNeo),
                new FinishGroundIntakeCommand(noteManagementSubsystem, groundIntakeSubsystem),
                followFirstTrajectory("twoNoteContestedAlt", "twoNoteContestedAlt-2", isNeo),
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem)
        );

        var threeNoteContested = new SequentialCommandGroup(
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem),
                followFirstTrajectory("threeNoteContested", "threeNoteContested-1", isNeo),
                new FinishGroundIntakeCommand(noteManagementSubsystem, groundIntakeSubsystem),
                followFirstTrajectory("threeNoteContested", "threeNoteContested-2", isNeo),
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem),
                followFirstTrajectory("threeNoteContested", "threeNoteContested-3", isNeo),
                new FinishGroundIntakeCommand(noteManagementSubsystem, groundIntakeSubsystem),
                followFirstTrajectory("threeNoteContested", "threeNoteContested-4", isNeo),
                new AchieveFlatShotSpeedCommand(shooterSubsystem, noteManagementSubsystem)
        );

        SendableChooser<Command> chooser = new SendableChooser<>();
        chooser.setDefaultOption("fourNote", fourNote); //TODO later
        chooser.addOption("oneNoteCollect", oneNoteCollect);
        chooser.addOption("twoNote", twoNote);
        chooser.addOption("shootLeave", shootLeave);
        chooser.addOption("twoNoteCollect", twoNoteCollect);
        chooser.addOption("threeNote", threeNote);
        chooser.addOption("fourNote", fourNote);
        chooser.addOption("mvpTaxi", mvpTaxi);
        chooser.addOption("shootOnly", shootOnly);
        chooser.addOption("rotationTest", rotationTest);
        chooser.addOption("twoNoteContested", twoNoteContested);
        chooser.addOption("twoNoteContestedAlt", twoNoteContestedAlt);
        chooser.addOption("threeNoteContested", threeNoteContested);


        SmartDashboard.putData("Path", chooser);
        return chooser;
    }

    void loadCommands() {

        //When driver
        Trigger xGreaterThan = operatorInput.driver.axisGreaterThan(XboxController.Axis.kLeftX.value, 0.1).or(operatorInput.driver.axisLessThan(XboxController.Axis.kLeftX.value, -0.1));
        Trigger yGreaterThan = operatorInput.driver.axisGreaterThan(XboxController.Axis.kLeftY.value, 0.1).or(operatorInput.driver.axisLessThan(XboxController.Axis.kLeftY.value, -0.1));
        Trigger rotGreaterThan = operatorInput.driver.axisGreaterThan(XboxController.Axis.kRightX.value, 0.1).or(operatorInput.driver.axisLessThan(XboxController.Axis.kRightX.value, -0.1));
        Trigger climberThreshold = operatorInput.operatorControl.axisGreaterThan(XboxController.Axis.kRightY.value, 0.1).or(operatorInput.driver.axisLessThan(XboxController.Axis.kRightY.value, -0.1));

        operatorInput.isTeleop.and(xGreaterThan.or(yGreaterThan).or(rotGreaterThan)).whileTrue(new AugmentedDriveCommand(SWERVE, driveSubsystem, odometrySubsystem, operatorInput));

        // Trigger thingsA
        operatorInput.ampSetpoint_hold.whileTrue(new PivotToAmpFireGroup(shooterSubsystem, noteManagementSubsystem, 100));
        operatorInput.speakerSetpoint_hold.whileTrue(new PivotToSpeakerFireGroup(shooterSubsystem, noteManagementSubsystem, 100));
        operatorInput.shootManually.whileTrue(new FeedFlywheelAndFireGroup(shooterSubsystem, noteManagementSubsystem, 100));
        operatorInput.setShooterAngleManually.onTrue(new ManualPivotCommand(operatorInput, shooterSubsystem));

        operatorInput.sourceIntake_hold.whileTrue(new PivotToSourceConsumeGroup(shooterSubsystem, noteManagementSubsystem, 100));
        operatorInput.groundIntakeHold.or(operatorInput.groundIntakeHoldOp).and(operatorInput.groundOuttakeHold.negate())
                .whileTrue(new FeedGroundIntakeGroup(noteManagementSubsystem, groundIntakeSubsystem));
        operatorInput.groundOuttakeHold.or(operatorInput.groundOuttakeHoldOp).and(operatorInput.groundIntakeHold.negate())
                .whileTrue(new GroundOuttakeCommand(groundIntakeSubsystem, noteManagementSubsystem));

        HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
                new PIDController(DRIVE_X_PID.pConstant(), DRIVE_X_PID.iConstant(), DRIVE_X_PID.dConstant()),
                new PIDController(DRIVE_Y_PID.pConstant(), DRIVE_Y_PID.iConstant(), DRIVE_Y_PID.dConstant()),
                new ProfiledPIDController(
                        DRIVE_T_PID.pConstant(),
                        DRIVE_T_PID.iConstant(),
                        DRIVE_T_PID.dConstant(),
                        new TrapezoidProfile.Constraints(2, 2)
                ) //TODO
        );

        operatorInput.autoAlignHold.whileTrue(new MoveToAlignCommand(driveSubsystem, visionSubsystem, holonomicDriveController, odometrySubsystem));
        operatorInput.isTeleop.and(climberThreshold).whileTrue(new MoveClimberCommand(climberSubsystem, operatorInput));


        operatorInput.resetGyroPress.onTrue(Commands.runOnce(() -> {
            odometrySubsystem.debugZero();
            odometrySubsystem.forceOdometryToThinkWeAreAt(new Pose3d(new Pose2d(0, 0, new Rotation2d())));
        }));

    }


    SwerveDriveKinematics loadKinematics() {
        return new SwerveDriveKinematics(
                ODO.fl_offset(),
                ODO.fr_offset(),
                ODO.bl_offset(),
                ODO.br_offset()
        );
    }

    DriveSubsystem loadDriveSubsystem() {
        SwerveModule[] modules = loadSwerveModules();

        return new DriveSubsystem(modules, kinematics);
    }

    SwerveModule[] loadSwerveModules() {
        SwerveModule[] modules = new SwerveModule[4];


        for (int i = 0; i < modules.length; i++) {
            SimpleMotorFeedforward ff = new SimpleMotorFeedforward(FF_SWERVE[i].ff_ks(), FF_SWERVE[i].ff_kv());
            ILinearVelocityController driveMotor;
            IRotationalController steerController;
            IRotationEncoder absoluteEncoder;

            if (DISABLER.drive_disabled()) {
                driveMotor = HardwareDisabled.linearMotor_velocityPID();
                steerController = HardwareDisabled.rotationalController_disabled();
                absoluteEncoder = HardwareDisabled.rotationEncoder_disabled();
            }
            else if (Robot.isSimulation()) {
                driveMotor = HardwareSIM.linearSIM_velocityPid(DRIVES[i], DRIVE_PIDS[i], DCMotor.getNEO(1));
                steerController = HardwareSIM.rotationalSIM_pid(STEERS[i], STEER_PIDS[i], DCMotor.getNEO(1));
                absoluteEncoder = steerController; //TODO silly hack wtf this is not a hack i have spent two hours on this and i have not found a solution
            } else {
                if (MattlibSettings.ROBOT == MattlibSettings.Robot.CARY) {
                    driveMotor = HardwareCTRE.linearFX_builtInVelocityPID(DRIVES[i], DRIVE_PIDS[i]);
                    steerController = HardwareREV.rotationalSpark_builtInPID(STEERS[i], STEER_PIDS[i]);
                    absoluteEncoder = HardwareUtil.thriftyEncoder(STEER_ABS_ENCODERS[i]);
                } else {
                    driveMotor = HardwareREV.linearSpark_builtInVelocityPID(DRIVES[i], DRIVE_PIDS[i]);
                    steerController = HardwareREV.rotationalSpark_builtInPID(STEERS[i], STEER_PIDS[i]);
                    absoluteEncoder = HardwareUtil.thriftyEncoder(STEER_ABS_ENCODERS[i]);
                }

            }
            /*
            if (disabled) {
                driveMotor = HardwareDisabled.linearMotor_disabled();
            }
*/
            modules[i] = new SwerveModule(driveMotor, steerController, absoluteEncoder, ff);
        }

        return modules;
    }

    ShooterSubsystem loadShooterSubsystem() {
        IRotationalController leftMotor;
        IRotationalController rightMotor;
        IRotationalController angleMotor;
        IRotationEncoder pivotEncoder;
        IRotationEncoder velocityEncoder;

        if (DISABLER.shooter_disabled()) {
            leftMotor = HardwareDisabled.rotationalController_disabled();
            rightMotor = HardwareDisabled.rotationalController_disabled();
            angleMotor = HardwareDisabled.rotationalController_disabled();
            pivotEncoder = HardwareDisabled.rotationEncoder_disabled();
            velocityEncoder = HardwareDisabled.rotationEncoder_disabled();
        } else if (Robot.isSimulation()) {
            leftMotor = HardwareSIM.rotationalSIM_pid(SHOOTER_WHEEL_1, SHOOTER_PID_1, DCMotor.getNEO(1));
            rightMotor = HardwareSIM.rotationalSIM_pid(SHOOTER_WHEEL_2, SHOOTER_PID_2, DCMotor.getNEO(1));
            angleMotor = HardwareSIM.rotationalSIM_pid(PIVOT, PIVOT_PID, DCMotor.getNEO(1));
            pivotEncoder = angleMotor;
            velocityEncoder = leftMotor; //TODO switch out leftMotor with actual velocity encoder

        } else {
            leftMotor = HardwareREV.rotationalSpark_builtInPID(SHOOTER_WHEEL_1, SHOOTER_PID_1);
            rightMotor = HardwareREV.rotationalSpark_builtInPID(SHOOTER_WHEEL_2, SHOOTER_PID_2);
            angleMotor = HardwareREV.rotationalSpark_builtInPID(PIVOT, PIVOT_PID);
            pivotEncoder = new ThriftyAbsoluteEncoder(
                    new AnalogInput(SHOOTER.pivotChannel_dio()), SHOOTER_ABSOLUTE);
            velocityEncoder = new ThroughBoreEncoder(
                    new Encoder(SHOOTER.velocityChannelA_dio(), SHOOTER.velocityChannelB_dio()), VELOCITY_ENCODER
            );
        }


        return new ShooterSubsystem(
                leftMotor,
                rightMotor,
                angleMotor,
                pivotEncoder,
                SHOOTER,
                SHOOTER_ABSOLUTE,
                velocityEncoder
        );

    }

    NoteManagementSubsystem loadNoteManagementSubsystem() {
        if (DISABLER.nms_disabled()) {
            return MockingUtil.buddy(NoteManagementSubsystem.class);
        }

        ILinearMotor nms_bottomMotor;
        ILinearMotor nms_topMotor;
        DigitalInput beamBreak = new DigitalInput(NMS.channel());

        if (DISABLER.nms_disabled()) {
            nms_bottomMotor = HardwareDisabled.linearController_disabled();
            nms_topMotor = HardwareDisabled.linearMotor_disabled();
        } else if (Robot.isSimulation()) {
            nms_bottomMotor = HardwareSIM.linearSIM_noPID(NMS_BOTTOMCOMPONENT, DCMotor.getNEO(1));
            nms_topMotor = HardwareSIM.linearSIM_noPID(NMS_TOPCOMPONENT, DCMotor.getNEO(1));
        } else {
            nms_bottomMotor = HardwareREV.linearSpark_noPID(NMS_BOTTOMCOMPONENT);
            nms_topMotor = HardwareREV.linearSpark_noPID(NMS_TOPCOMPONENT);
        }

        return new NoteManagementSubsystem(
                nms_bottomMotor, nms_topMotor, beamBreak,
                NMS);

    }

    OdometrySubsystem loadOdometrySubsystem() {

        IGyro gyro;

        if (Robot.isSimulation() || DISABLER.odometry_disabled()) {
            gyro = new KinematicGyro(driveSubsystem, kinematics);
        } else {
            Pigeon2 pigeon2 = new Pigeon2(SWERVE.pigeonCanId());

            gyro = new Pigeon2Gyro(pigeon2);
        }

        // TODO implement swappable version per condition (sim, disable, enable)

        return new OdometrySubsystem(
                driveSubsystem,
                visionSubsystem,
                new SwerveDrivePoseEstimator( //The auto path will reset all of this data anyways
                        kinematics,
                        new Rotation2d(),
                        new SwerveModulePosition[] { new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition(), new SwerveModulePosition()},
                        new Pose2d()
                ),
                gyro,
                kinematics,
                ODO
        ); //TODO
    }

    VisionSubsystem loadVisionSubsystem() {
        if (DISABLER.vision_disabled()) {
            return MockingUtil.buddy(VisionSubsystem.class);
        }

        PhotonCamera camera1;
        PhotonCamera camera2;
        camera1 = new PhotonCamera(CAMERAS.camera1Name());
        camera2 = new PhotonCamera(CAMERAS.camera2Name());

        AprilTagFieldLayout aprilTagFieldLayout;
        String filepath = Filesystem.getDeployDirectory().getPath() + "/2024-crescendo.json";

        // better error catching later ig
        try {
            aprilTagFieldLayout = new AprilTagFieldLayout(filepath);
        } catch (IOException e) {
            throw new IllegalStateException(e.getMessage() + " here is why");
        }

        // USE MATTLIB FOR CONSTANTS HERE IM JUST LAZY TODO
        Transform3d robotToCam1 = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));
        // using multi tag localization
        PhotonPoseEstimator photonPoseEstimator1 = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera1,
                robotToCam1
        );


        Transform3d robotToCam2 = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));
        PhotonPoseEstimator photonPoseEstimator2 = new PhotonPoseEstimator(
                aprilTagFieldLayout,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                camera2,
                robotToCam2

        );

        photonPoseEstimator1.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);
        photonPoseEstimator2.setMultiTagFallbackStrategy(PhotonPoseEstimator.PoseStrategy.LOWEST_AMBIGUITY);


        return new VisionSubsystem(
                camera1,
                camera2,
                aprilTagFieldLayout,
                photonPoseEstimator1,
                photonPoseEstimator2,
                new AprilTagDetector()
        );

    }

    ClimberSubsystem loadClimberSubsystem() {
        ILinearController leftClimber;
        ILinearController rightClimber;
        SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(CLIMBER.ff_ks(), CLIMBER.ff_kv());
        if (DISABLER.climber_disabled()) {
            leftClimber = HardwareDisabled.linearController_disabled();
            rightClimber = HardwareDisabled.linearController_disabled();
        } else {
            leftClimber = HardwareREV.linearSpark_builtInPID(LEFT_CLIMBER, CLIMBER_PID);
            rightClimber = HardwareREV.linearSpark_builtInPID(RIGHT_CLIMBER, CLIMBER_PID);
        }

        return new ClimberSubsystem(
                leftClimber,
                rightClimber,
                feedForward
        ); // TODO

    }

    GroundIntakeSubsystem loadGroundIntakeSubsystem() {
        ILinearController topGroundIntake;
        ILinearController bottomGroundIntake;
        SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(GROUND_INTAKE.ff_ks(), GROUND_INTAKE.ff_kv());

        if (DISABLER.groundIntake_disabled()) {
            topGroundIntake = HardwareDisabled.linearController_disabled();
            bottomGroundIntake = HardwareDisabled.linearController_disabled();

        } else if (Robot.isSimulation()) {
            topGroundIntake = HardwareSIM.linearSIM_pid(GROUND_INTAKE_TOP, GROUND_INTAKE_PID, DCMotor.getNeo550(1));
            bottomGroundIntake = HardwareSIM.linearSIM_pid(GROUND_INTAKE_BOTTOM, GROUND_INTAKE_PID, DCMotor.getNeo550(1));
        } else {
            topGroundIntake = HardwareREV.linearSpark_builtInPID(GROUND_INTAKE_TOP, GROUND_INTAKE_PID);
            bottomGroundIntake = HardwareREV.linearSpark_builtInPID(GROUND_INTAKE_BOTTOM, GROUND_INTAKE_PID);
        }

        return new GroundIntakeSubsystem(
                topGroundIntake,
                bottomGroundIntake,
                feedForward
        );
    }


    //Components MUST be created in the Robot class (because of how static bs works)

    //generator stuff
    public static final ConsoleComponent CONSOLE = LOG.load(ConsoleComponent.class, "console");
    public static final GenerateFFComponent SHOOTER_WHEEL_2_FFGEN = LOG.load(GenerateFFComponent.class, "shooter/wheel_2_ffgen");
    public static final GenerateFFComponent[] DRIVE_MOTORS_FFGEN = LOG.loadRange(GenerateFFComponent.class, "swerve/ffgen",4, Util.RENAMER);

    //config stuff
    public static final VisionComponent VISION = LOG.load(VisionComponent.class, "vision");

    public static final ClimberComponent CLIMBER = LOG.load(ClimberComponent.class, "climber");
    public static final PIDComponent CLIMBER_PID = LOG.load(PIDComponent.class, "climber/climber_pid");
    public static final MotorComponent LEFT_CLIMBER = LOG.load(MotorComponent.class, "climber/left");
    public static final MotorComponent RIGHT_CLIMBER = LOG.load(MotorComponent.class, "climber/right");

    public static final GroundIntakeComponent GROUND_INTAKE = LOG.load(GroundIntakeComponent.class, "ground_intake");
    public static final MotorComponent GROUND_INTAKE_TOP = LOG.load(MotorComponent.class, "ground_intake/top");
    public static final MotorComponent GROUND_INTAKE_BOTTOM = LOG.load(MotorComponent.class, "ground_intake/bottom");
    public static final PIDComponent GROUND_INTAKE_PID = LOG.load(PIDComponent.class, "ground_intake/pid");

    public static final ShooterComponent SHOOTER = LOG.load(ShooterComponent.class, "shooter");
    public static final MotorComponent SHOOTER_WHEEL_1 = LOG.load(MotorComponent.class, "shooter/wheel_1");
    public static final MotorComponent SHOOTER_WHEEL_2 = LOG.load(MotorComponent.class, "shooter/wheel_2");
    public static final PIDComponent SHOOTER_PID_1 = LOG.load(PIDComponent.class, "shooter/wheel_1/pid");
    public static final PIDComponent SHOOTER_PID_2 = LOG.load(PIDComponent.class, "shooter/wheel_2/pid");
    public static final AbsoluteEncoderComponent SHOOTER_ABSOLUTE = LOG.load(AbsoluteEncoderComponent.class, "shooter/absolute");
    public static final AbsoluteEncoderComponent VELOCITY_ENCODER = LOG.load(AbsoluteEncoderComponent.class, "shooter/velocity");
    public static final MotorComponent PIVOT = LOG.load(MotorComponent.class, "shooter/pivot");
    public static final PIDComponent PIVOT_PID = LOG.load(PIDComponent.class, "shooter/pivot/pid");

    public static final NoteManagementComponent NMS = LOG.load(NoteManagementComponent.class, "nms");
    public static final MotorComponent NMS_TOPCOMPONENT = LOG.load(MotorComponent.class, "nms/top");
    public static final MotorComponent NMS_BOTTOMCOMPONENT = LOG.load(MotorComponent.class, "nms/bottom");

    public static final FFComponent[] FF_SWERVE = LOG.loadRange(FFComponent.class, "swerve/ff", 4, Util.RENAMER);
    public static final SwerveComponent SWERVE = LOG.load(SwerveComponent.class, "swerve");
    public static final OdometrySubsystem.Component ODO = LOG.load(OdometrySubsystem.Component.class, "odometry");
    public static final CommonMotorComponent STEER_COMMON = LOG.load(CommonMotorComponent.class, "swerve/steer_common");
    public static final CommonPIDComponent STEER_PID_COMMON = LOG.load(CommonPIDComponent.class, "swerve/steer_pid_common");
    public static final CommonPIDComponent DRIVE_PID_COMMON = LOG.load(CommonPIDComponent.class, "swerve/drive_pid_common");

    public static final MotorComponent[] DRIVES = LOG.loadRange(MotorComponent.class, "swerve/drive", 4, Util.RENAMER);
    public static final MotorComponent[] STEERS = MotorComponent.ofRange(STEER_COMMON, LOG.loadRange(IndividualMotorComponent.class, "swerve/steer", 4, Util.RENAMER));
    public static final PIDComponent[] STEER_PIDS = PIDComponent.ofRange(STEER_PID_COMMON, LOG.loadRange(IndividualPIDComponent.class, "swerve/steer_pid", 4, Util.RENAMER));
    public static final PIDComponent[] DRIVE_PIDS = PIDComponent.ofRange(DRIVE_PID_COMMON, LOG.loadRange(IndividualPIDComponent.class, "swerve/drive_pid", 4, Util.RENAMER));

    public static final AbsoluteEncoderComponent[] STEER_ABS_ENCODERS = LOG.loadRange(AbsoluteEncoderComponent.class, "swerve/abs", 4, Util.RENAMER);

    public static final PIDComponent DRIVE_X_PID = LOG.load(PIDComponent.class, "swerve/x_holonomic_pid");
    public static final PIDComponent DRIVE_Y_PID = LOG.load(PIDComponent.class, "swerve/y_holonomic_pid");
    public static final PIDComponent DRIVE_T_PID = LOG.load(PIDComponent.class, "swerve/t_holonomic_pid");

    public static final CamerasComponent CAMERAS = LOG.load(CamerasComponent.class, "cameras");
    public static final DisablerComponent DISABLER = LOG.load(DisablerComponent.class, "disabler");


}
