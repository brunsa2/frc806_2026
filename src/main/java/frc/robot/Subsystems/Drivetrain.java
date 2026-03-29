package frc.robot.Subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static edu.wpi.first.wpilibj2.command.Commands.none;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.run;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;

public class Drivetrain extends SubsystemBase {
    private CommandXboxController controller;
    private boolean isWaitingToCalibrate;
    private Pigeon2 imu;
    public SwerveModule[] modules;
    private SwerveDriveKinematics kinematics;
    private SwerveDriveOdometry odometry;
    private ChassisSpeeds m_chassisSpeeds;
    private double translationMaxAccelerationMetersPerSecondSquared = 25;
    private double rotationMaxAccelerationRadiansPerSecondSquared = 50;
    private SlewRateLimiter translationXLimiter = new SlewRateLimiter(translationMaxAccelerationMetersPerSecondSquared);
    private SlewRateLimiter translationYLimiter = new SlewRateLimiter(translationMaxAccelerationMetersPerSecondSquared);
    private SlewRateLimiter rotationLimiter = new SlewRateLimiter(rotationMaxAccelerationRadiansPerSecondSquared);
    private final StructArrayPublisher<SwerveModuleState> statePublisher;
    private Pose pose;
    // TODO: these motional profiles are front/back and left/right which means going
    // on a diagonal can go faster. We need a 2D aware motion profiler (this might be provided
    // by custom path planning later on...)
    private final TrapezoidProfile lateralProfile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(0.05, 0.1));
    private final TrapezoidProfile angularProfile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(0.05, 0.1));
    private TrapezoidProfile.State forwardProfileState;
    private TrapezoidProfile.State sidewaysProfileState;
    private TrapezoidProfile.State angularProfileState;
    // private final PIDController visionForwardBackController = new PIDController(2.5, 0, 0);
    private final ProfiledPIDController visionForwardBackController = new ProfiledPIDController(
        4, 0, 0,
        new TrapezoidProfile.Constraints(3, 5));
    private final ProfiledPIDController visionSidewaysController = new ProfiledPIDController(
        4, 0, 0,
        new TrapezoidProfile.Constraints(3, 5));
    private final PIDController visionRotationsController = new PIDController(0, 0, 0);

    private final Alert willCalibrateAlert = new Alert("Robot will enter drivetrain calibration when re-enabled", AlertType.kInfo);
    private final Alert calibratingAlert = new Alert("Drivetrain can be calibrated. Align wheels when disabled and calibrate or cancel", AlertType.kInfo);

    private double timestamp;
    
    public Drivetrain(SwerveModule[] modules, CommandXboxController controller) {
        imu = new Pigeon2(Constants.PigeonID, new CANBus("*"));
        this.modules = modules;
        kinematics = new SwerveDriveKinematics(Constants.moduleLocations);
        odometry = new SwerveDriveOdometry(kinematics, getGyroscopeRotation(), getModulePositions());
        statePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("SwerveModules", SwerveModuleState.struct).publish();
        setDefaultCommand(manualDriveFieldRelative());
        // TODO: We should inject axes rather than an entire controller
        this.controller = controller;

        SmartDashboard.putData(enableCalibration());
        SmartDashboard.putData(calibrate());
        SmartDashboard.putData(cancelCalibration());
        SmartDashboard.putData(this);
    }

    public void setPose(Pose pose) {
        this.pose = pose;
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(imu.getYaw().getValueAsDouble());
    }

    public void resetGyro() {
        imu.reset();
    }

    public Command getInitialCommand() {
        if (Preferences.getBoolean("Drivetrain.enableDrivetrainCalibration", true)) {
            return waitToCalibrate();
        } else {
            return getDefaultCommand();
        }
    }

    public Command enableCalibration() {
        return runOnce(() -> { willCalibrateAlert.set(true); Preferences.setBoolean("Drivetrain.enableDrivetrainCalibration", true); }).withName("Enabling calibration");
    }

    public Command waitToCalibrate() {
        return runOnce(() -> { isWaitingToCalibrate = true; willCalibrateAlert.set(false); calibratingAlert.set(true); } ).andThen(run(() -> {}))
        .finallyDo(() -> {
            Preferences.setBoolean("Drivetrain.enableDrivetrainCalibration", false);
            willCalibrateAlert.set(false);
            calibratingAlert.set(false);
        }).withName("Waiting to calibrate");
    }
    
    public Command calibrate() {
        return parallel(
            runOnce(() -> isWaitingToCalibrate = false),
            modules[0].calibrate(),
            modules[1].calibrate(),
            modules[2].calibrate(),
            modules[3].calibrate()
        ).onlyIf(() -> isWaitingToCalibrate).withName("Calibrating");
    }

    public Command cancelCalibration() {
        return runOnce(() -> isWaitingToCalibrate = false).withName("Canceling calibration");
    }

    public Command manualDrive() {
        return run(() -> setSpeeds(getManualChassisSpeeds())).withName("Drive");
    }

    public Command manualDriveFieldRelative() {
        return run(() -> setFieldRelativeSpeeds(getManualChassisSpeeds())).withName("Drive Field Relative");
    }

    public Command alignToTag(int tagId, Translation2d offset, Rotation2d theta) {
        var tag = Constants.Pose.FieldLayout.getTagPose(tagId);
        return tag.map(t -> {
            var tagPose2d = t.toPose2d();
            var tagToTarget = new Transform2d(offset.unaryMinus(), theta);
            var targetPose = tagPose2d.transformBy(tagToTarget);

            return runOnce(() -> {
                var initialPose = pose.getPose();
                forwardProfileState = new TrapezoidProfile.State(initialPose.getX(), 0.0);
                sidewaysProfileState = new TrapezoidProfile.State(initialPose.getY(), 0.0);
                angularProfileState = new TrapezoidProfile.State(initialPose.getRotation().getRotations(), 0.0);

                timestamp = getFPGATimestamp();

                SmartDashboard.putNumber("FP0", initialPose.getX());
                visionForwardBackController.reset(initialPose.getX());
                visionSidewaysController.reset(initialPose.getY());
            }).andThen(run(()-> {
                var robotPose = pose.getPose();

                double newTimestamp = getFPGATimestamp();
                double elapsedTime = newTimestamp - timestamp;
                SmartDashboard.putNumber("ET", elapsedTime);
                timestamp = newTimestamp;

                forwardProfileState = lateralProfile.calculate(elapsedTime, forwardProfileState, new TrapezoidProfile.State(targetPose.getX(), 0));
                // sidewaysProfileState = lateralProfile.calculate(elapsedTime, sidewaysProfileState, new TrapezoidProfile.State(targetPose.getY(), 0));
                // // Note: this might not work for angular since it doesn't support continuous motion
                // angularProfileState = angularProfile.calculate(elapsedTime, angularProfileState, new TrapezoidProfile.State(robotPose.getRotation().getRotations(), 0));
                
                var forwardVelocity = -visionForwardBackController.calculate(robotPose.getX(), targetPose.getX());
                var sidewaysVelocity = -visionSidewaysController.calculate(robotPose.getY(), targetPose.getY());
                var angularVelcoity = -visionRotationsController.calculate(robotPose.getRotation().getRotations(), targetPose.getRotation().getRotations());

                var speeds = new ChassisSpeeds(forwardVelocity, sidewaysVelocity, angularVelcoity);

                setSpeeds(speeds);
            }));
        }).orElse(new SequentialCommandGroup());
    }

    public Command manualDriveFieldRelativeWithSteerAimToNearestAprilTag() {
        return none().withName("Drive Field Relative with Steer Aim to Nearest April Tag");
    }

    @Override
    public void periodic() {
        // TODO: calibration triggers don't need to be commands
        if (controller.start().getAsBoolean()){
            resetGyro();
        }
    }

    private ChassisSpeeds getManualChassisSpeeds() {
        double x = controller.getLeftX(), y = controller.getLeftY(), theta = controller.getRightX();
        
        if (Math.hypot(x, y) < Constants.Drivetrain.ControllerDeadband) {
            x = 0;
            y = 0;
        }

        if (Math.abs(theta) < Constants.Drivetrain.ControllerDeadband) {
            theta = 0.0;
        }

        x = (x > 0) ? Math.abs(Math.pow(x, Constants.Drivetrain.TranslationPow)) : -Math.abs(Math.pow(x, Constants.Drivetrain.TranslationPow));
        y = (y > 0) ? Math.abs(Math.pow(y, Constants.Drivetrain.TranslationPow)) : -Math.abs(Math.pow(y, Constants.Drivetrain.TranslationPow));
        theta = (theta > 0) ? Math.abs(Math.pow(theta, Constants.Drivetrain.RotationPow)) : -Math.abs(Math.pow(theta, Constants.Drivetrain.RotationPow));

        double slowModeFactor = (controller.getLeftTriggerAxis() * Constants.Drivetrain.SlowFactor) + Constants.Drivetrain.SlowFactorOffset;

        return new ChassisSpeeds(
            (y * Constants.attainableMaxTranslationalSpeedMPS) / slowModeFactor, 
            (x * Constants.attainableMaxTranslationalSpeedMPS) / slowModeFactor, 
            -(theta * Constants.attainableMaxRotationalVelocityRPS) / slowModeFactor
        );
    }

    public void setSpeeds(ChassisSpeeds chassisSpeeds) {
        setModuleTargetStates(chassisSpeeds, new Translation2d());
    }

	public void setSpeeds(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation) {
        setModuleTargetStates(chassisSpeeds, centerOfRotation);
    }

    public void setFieldRelativeSpeeds(ChassisSpeeds chassisSpeeds) {
        chassisSpeeds.vxMetersPerSecond = translationXLimiter.calculate(chassisSpeeds.vxMetersPerSecond);
        chassisSpeeds.vyMetersPerSecond = translationYLimiter.calculate(chassisSpeeds.vyMetersPerSecond);
        chassisSpeeds.omegaRadiansPerSecond = rotationLimiter.calculate(chassisSpeeds.omegaRadiansPerSecond);

        setModuleTargetStates(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getGyroscopeRotation()));
    }

    public void setModuleTargetStates(ChassisSpeeds chassisSpeeds) {
        setModuleTargetStates(chassisSpeeds, new Translation2d());
    }

    public void setModuleTargetStates(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation) {
        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(chassisSpeeds, centerOfRotation);
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.attainableMaxModuleSpeedMPS);
        for (int i = 0; i < modules.length; ++i) {
            targetStates[i].optimize(Rotation2d.fromRotations(modules[i].getModuleAngRotations()));
            modules[i].setTargetState(targetStates[i]);
        }
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] {modules[0].getModulePosition(),modules[1].getModulePosition(),modules[2].getModulePosition(),modules[3].getModulePosition()};
    }

    public ChassisSpeeds getChasisSpeed() {
        return kinematics.toChassisSpeeds(
            modules[0].getSwerveModuleState(),
            modules[1].getSwerveModuleState(),
            modules[2].getSwerveModuleState(),
            modules[3].getSwerveModuleState()
        );
    }
}
