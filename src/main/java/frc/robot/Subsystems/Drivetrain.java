package frc.robot.Subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Commands.DriveFieldRelative;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class Drivetrain extends SubsystemBase {
    boolean isWaitingToCalibrate;
    Pigeon2 IMU;
    public SwerveModule[] modules;
    SwerveDriveKinematics kinematics;
    SwerveDriveOdometry odometry;
    ChassisSpeeds m_chassisSpeeds;
    double translationMaxAccelerationMetersPerSecondSquared = 25;
    double rotationMaxAccelerationRadiansPerSecondSquared = 50;
    SlewRateLimiter translationXLimiter = new SlewRateLimiter(translationMaxAccelerationMetersPerSecondSquared);
    SlewRateLimiter translationYLimiter = new SlewRateLimiter(translationMaxAccelerationMetersPerSecondSquared);
    SlewRateLimiter rotationLimiter = new SlewRateLimiter(rotationMaxAccelerationRadiansPerSecondSquared);
    private final StructArrayPublisher<SwerveModuleState> statePublisher;
    private int fiducialIdTarget = -1;

    // PhotonCamera camera = new PhotonCamera("photonvision");
    // SwerveDrivePoseEstimator aimingPoseEstimator;
    // double currentVisionTime = 0;
    // double lastVisionTime = 0;
    private final PIDController visionForwardBackController = new PIDController(2.5, 0, 0);
    private final PIDController visionSidewaysController = new PIDController(2.5, 0, 0);
    private final PIDController visionRotationsController = new PIDController(0, 0, 0);

    // private final Alert calibratingAlert = new Alert("Calibrating steering motors", AlertType.kInfo);
    private final Alert willCalibrateAlert = new Alert("Robot will enter drivetrain calibration when re-enabled", AlertType.kInfo);
    private final Alert calibratingAlert = new Alert("Drivetrain can be calibrated. Align wheels when disabled and calibrate or cancel", AlertType.kInfo);
    private Pose pose;
    
    public Drivetrain(SwerveModule[] modules, CommandXboxController controller) {
        IMU = new Pigeon2(Constants.PigeonID, new CANBus("*"));
        this.modules = modules;
        kinematics = new SwerveDriveKinematics(Constants.moduleLocations);
        odometry = new SwerveDriveOdometry(kinematics, getGyroscopeRotation(), getModulePositions());
        statePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("SwerveModules", SwerveModuleState.struct).publish();
        setDefaultCommand(new DriveFieldRelative(this, controller));
        // setDefaultCommand();

        SmartDashboard.putData(calibrate());
        SmartDashboard.putData(this);

        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
            this::getPose,
            this::resetPose,
            this::getChasisSpeed,
            (speeds, feedforwards) -> setModuleTargetStates(speeds),

            new PPHolonomicDriveController(
                new PIDConstants(Constants.Drivetrain.SpeedKP, Constants.Drivetrain.SpeedKI, Constants.Drivetrain.SpeedKD),
                new PIDConstants(Constants.Drivetrain.SteerKP, Constants.Drivetrain.SteerKI, Constants.Drivetrain.SteerKD)
            ),
            config,
            () -> {
                var alliance = DriverStation.getAlliance();
                if (!alliance.isPresent()) {
                return false;
                }
                return alliance.get() == DriverStation.Alliance.Red;
            },
            this
        );
    }

    public void setPose(Pose pose) {
        this.pose = pose;
    }

	public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(IMU.getYaw().getValueAsDouble());
    }

    public void resetGyro() {
        IMU.reset();
    }

    private Pose2d getPose() {
        return pose.getPose();
    }

    private void resetPose(Pose2d pose) {
        // Vision system reliably gives starting pose, so we ignore this one
    }
    
    public Command calibrate() {
        return parallel(
            runOnce(() -> {isWaitingToCalibrate = false; System.out.println("Swerve calibration triggered");}),
            modules[0].calibrate(),
            modules[1].calibrate(),
            modules[2].calibrate(),
            modules[3].calibrate()
        ).ignoringDisable(true).withName("Calibrating");
    }

    public Command cancelCalibration() {
        return runOnce(() -> isWaitingToCalibrate = false ).withName("Canceling calibration");
    }
    
    public Command aimAtTag(int tagId, Translation2d offset, Rotation2d theta) {
        var tag = Constants.Pose.FieldLayout.getTagPose(tagId);
        return tag.map(t -> {
            var tagPose2d = t.toPose2d();
            var tagToTarget = new Transform2d(offset.unaryMinus(), theta);
            var targetPose = tagPose2d.transformBy(tagToTarget);

            SmartDashboard.putNumber("TX", targetPose.getX());
            SmartDashboard.putNumber("TY", targetPose.getY());
            SmartDashboard.putNumber("TT", targetPose.getRotation().getDegrees());

            return run(()-> {
                var robotPose = getPose();

                var forwardVelocity = -visionForwardBackController.calculate(robotPose.getX(), targetPose.getX());
                var sidewaysVelocity = -visionSidewaysController.calculate(robotPose.getY(), targetPose.getY());
                var angularVelcoity = visionRotationsController.calculate(robotPose.getRotation().getRotations(), targetPose.getRotation().getRotations());

                var speeds = new ChassisSpeeds(forwardVelocity, sidewaysVelocity, angularVelcoity);

                drive(speeds);
                SmartDashboard.putNumber("F", forwardVelocity);
                SmartDashboard.putNumber("S", sidewaysVelocity);
                SmartDashboard.putNumber("A", angularVelcoity);
            });
        }).orElse(Commands.none());
    }

    public void drive(ChassisSpeeds chassisSpeeds){
        setModuleTargetStates(chassisSpeeds, new Translation2d());
    }

    public void drive(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation) {
        setModuleTargetStates(chassisSpeeds, centerOfRotation);
    }

    public void driveFieldRelative(ChassisSpeeds chassisSpeeds) {
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

    @Override
    public void periodic() {}
}
