package frc.robot.Subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
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
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
// import edu.wpi.first.wpilibj.ADIS16470_IMU;
// import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.race;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;

import java.util.function.Function;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Commands.DriveFieldRelative;

public class Drivetrain extends SubsystemBase {

    // ADIS16470_IMU IMU;
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

    private Supplier<Pose2d> pose2dSupplier;


    // private final Alert calibratingAlert = new Alert("Calibrating steering motors", AlertType.kInfo);
    private final Alert willCalibrateAlert = new Alert("Robot will enter drivetrain calibration when re-enabled", AlertType.kInfo);
    private final Alert calibratingAlert = new Alert("Drivetrain can be calibrated. Align wheels when disabled and calibrate or cancel", AlertType.kInfo);
    
    public Drivetrain(SwerveModule[] modules, CommandXboxController controller) {
        // IMU = new ADIS16470_IMU();
        IMU = new Pigeon2(Constants.PigeonID, new CANBus("*"));
        this.modules = modules;
        kinematics = new SwerveDriveKinematics(Constants.moduleLocations);
        odometry = new SwerveDriveOdometry(kinematics, getGyroscopeRotation(), getModulePositions());
        statePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("SwerveModules", SwerveModuleState.struct).publish();
        setDefaultCommand(new DriveFieldRelative(this, controller));
        // setDefaultCommand();

        SmartDashboard.putData(enableCalibration());
        SmartDashboard.putData(calibrate());
        SmartDashboard.putData(cancelCalibration());
        SmartDashboard.putData(this);

        
    }

    public void setPose2dSupplier(Supplier<Pose2d> pose2dSupplier) {
        this.pose2dSupplier = pose2dSupplier;
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public Rotation2d getGyroscopeRotation() {
        //return Rotation2d.fromDegrees(IMU.get());
    
        // return Rotation2d.fromDegrees(-IMU.getAngle(IMUAxis.kY));
        // return Rotation2d.fromDegrees(IMU.getAngle());
        return Rotation2d.fromDegrees(IMU.getYaw().getValueAsDouble());
    }

    // public void calibrateGyro(){
    //     IMU.calibrate();   
    //  }

    public void resetGyro() {
        IMU.reset();
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
            runOnce(() -> isWaitingToCalibrate = false ),
            modules[0].calibrate(),
            modules[1].calibrate(),
            modules[2].calibrate(),
            modules[3].calibrate()
        ).onlyIf(() -> isWaitingToCalibrate).withName("Calibrating");
    }

    public Command cancelCalibration() {
        return runOnce(() -> isWaitingToCalibrate = false ).withName("Canceling calibration");
    }
    
    public Command aimAtTag(int tagId, Translation2d offset, Rotation2d theta) {
        var tag = Constants.Camera.FieldLayout.getTagPose(tagId);
        return tag.map(t -> {
            var tagPose2d = t.toPose2d();
            var tagToTarget = new Transform2d(offset.unaryMinus(), theta);
            var targetPose = tagPose2d.transformBy(tagToTarget);

            SmartDashboard.putNumber("TX", targetPose.getX());
            SmartDashboard.putNumber("TY", targetPose.getY());
            SmartDashboard.putNumber("TT", targetPose.getRotation().getDegrees());

            return run(()-> {
                var robotPose = pose2dSupplier.get();

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

    public void drive(ChassisSpeeds chassisSpeeds, Translation2d centerOfRotation){
        setModuleTargetStates(chassisSpeeds, centerOfRotation);
    }

    public void driveFieldRelative(ChassisSpeeds chassisSpeeds){
        chassisSpeeds.vxMetersPerSecond = translationXLimiter.calculate(chassisSpeeds.vxMetersPerSecond);
        chassisSpeeds.vyMetersPerSecond = translationYLimiter.calculate(chassisSpeeds.vyMetersPerSecond);
        chassisSpeeds.omegaRadiansPerSecond = rotationLimiter.calculate(chassisSpeeds.omegaRadiansPerSecond);

        // SmartDashboard.putNumber("gyro", getGyroscopeRotation().getDegrees());

        setModuleTargetStates(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getGyroscopeRotation()));
        // setModuleTargetStates(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getGyroscopeRotation()));
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

    public SwerveModulePosition[] getModulePositions(){
        return new SwerveModulePosition[] {modules[0].getModulePosition(),modules[1].getModulePosition(),modules[2].getModulePosition(),modules[3].getModulePosition()};
    }

    public ChassisSpeeds getChasisSpeed(){
        return kinematics.toChassisSpeeds(
            modules[0].getSwerveModuleState(),
            modules[1].getSwerveModuleState(),
            modules[2].getSwerveModuleState(),
            modules[3].getSwerveModuleState()
        );
    }

    // public void periodic() {
            
    //     var results = camera.getAllUnreadResults();
    //     for (var result: results) {
    //         if (!result.hasTargets()) {
    //             continue;
    //         }

    //         var bestTarget = result.getBestTarget();
    //         if (bestTarget == null) {
    //             return;
    //         }

    //         var translation = bestTarget.getBestCameraToTarget();

    //         if (Math.abs(-translation.getX() - aimingPoseEstimator.getEstimatedPosition().getX()) > Constants.Drivetrain.Vision.XRejectDistance ||
    //                 Math.abs(-translation.getY() - aimingPoseEstimator.getEstimatedPosition().getY()) > Constants.Drivetrain.Vision.XRejectDistance) {
    //             return;
    //         }

    //         lastVisionTime = currentVisionTime;
    //         currentVisionTime = result.getTimestampSeconds();

    //         aimingPoseEstimator.addVisionMeasurement(new Pose2d(-translation.getX(), -translation.getY(), new Rotation2d(-bestTarget.getYaw())), currentVisionTime);
    //     }
    // }

    public Command aim(double targetX, double targetY, double targetTheta) {
        // return runOnce(() -> {
        //     fiducialIdTarget = closestFiducialIdSupplier.get();
        // }).andThen(run(() -> {
        //     var poseEstimate = pose2dSupplier.apply(fiducialIdTarget);
        //     var forwardVelocity = MathUtil.clamp(visionForwardBackController.calculate(poseEstimate.getX(), targetX), -0.1, 0.1);
        //     var sidewaysVelocity = MathUtil.clamp(visionSidewaysController.calculate(poseEstimate.getY(), targetY), -0.1, 0.1);
        //     var angularVelcoity = MathUtil.clamp(visionRotationsController.calculate(poseEstimate.getRotation().getRadians(), targetTheta), -0.1, 0.1);
        //     var speeds = new ChassisSpeeds(forwardVelocity, sidewaysVelocity, angularVelcoity);

        //     // TODO: alert if targeting is running on odometry only
        //     drive(speeds, Constants.Camera.Position);
        // })).until(() -> {
        //     return visionForwardBackController.atSetpoint() &&
        //         visionSidewaysController.atSetpoint() &&
        //         visionRotationsController.atSetpoint();
        return Commands.none();
        
    }

    // public Command aim() {
    //     return run(() -> {
    //         var results = camera.getAllUnreadResults();
    //         if (results.isEmpty()) {
    //             return;
    //         }
    //         var result = results.get(results.size() - 1);
    //         if (!result.hasTargets()) {
    //             return;
    //         }
    //         var bestTarget = result.getBestTarget();
    //         if (bestTarget == null) {
    //             return;
    //         }
    //         var translation = bestTarget.getBestCameraToTarget();

    //         aimingPoseEstimator = new SwerveDrivePoseEstimator(
    //             kinematics,
    //             getGyroscopeRotation(),
    //             getModulePositions(),
    //             new Pose2d(-translation.getX(), -translation.getY(), new Rotation2d(-bestTarget.getYaw())),
    //             new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[]{Constants.Drivetrain.Odometry.PositionStdDev, Constants.Drivetrain.Odometry.PositionStdDev, Constants.Drivetrain.Odometry.AngleStdDev}),
    //             new Matrix<N3, N1>(Nat.N3(), Nat.N1(), new double[]{Constants.Drivetrain.Vision.XConstantStdDev, Constants.Drivetrain.Vision.YConstantStdDev, Constants.Drivetrain.Vision.AngleStdDev})
    //         );
    //         lastVisionTime = getFPGATimestamp();
    //     })
    //     .until(() -> aimingPoseEstimator != null).withTimeout(Constants.Drivetrain.Vision.InitialTimeoutSeconds).withName("Acquire target")
    //     .andThen(
    //         race(
    //             run(() -> {
    //                 aimingPoseEstimator.updateWithTime(getFPGATimestamp(), getGyroscopeRotation(), getModulePositions());
    //                 var poseEstimate = aimingPoseEstimator.getEstimatedPosition();
    //                 var forwardVelocity = MathUtil.clamp(visionForwardBackController.calculate(poseEstimate.getX(), Constants.Drivetrain.Vision.TargetX), -0.1, 0.1);
    //                 var sidewaysVelocity = MathUtil.clamp(visionSidewaysController.calculate(poseEstimate.getY(), Constants.Drivetrain.Vision.TargetY), -0.1, 0.1);
    //                 var angularVelcoity = MathUtil.clamp(visionRotationsController.calculate(poseEstimate.getRotation().getRadians(), Constants.Drivetrain.Vision.TargetTheta), -0.1, 0.1);
    //                 var speeds = new ChassisSpeeds(forwardVelocity, sidewaysVelocity, angularVelcoity);
    //                 drive(speeds);
    //             }).until(() -> visionForwardBackController.atSetpoint() &&
    //                     visionSidewaysController.atSetpoint() &&
    //                     visionRotationsController.atSetpoint()).withName("Drive"),
    //             run(() -> {
    //                 var results = camera.getAllUnreadResults();
    //                 for (var result: results) {
    //                     if (!result.hasTargets()) {
    //                         continue;
    //                     }

    //                     var bestTarget = result.getBestTarget();
    //                     if (bestTarget == null) {
    //                         return;
    //                     }

    //                     var translation = bestTarget.getBestCameraToTarget();

    //                     if (Math.abs(-translation.getX() - aimingPoseEstimator.getEstimatedPosition().getX()) > Constants.Drivetrain.Vision.XRejectDistance ||
    //                             Math.abs(-translation.getY() - aimingPoseEstimator.getEstimatedPosition().getY()) > Constants.Drivetrain.Vision.XRejectDistance) {
    //                         return;
    //                     }

    //                     lastVisionTime = currentVisionTime;
    //                     currentVisionTime = result.getTimestampSeconds();

    //                     aimingPoseEstimator.addVisionMeasurement(new Pose2d(-translation.getX(), -translation.getY(), new Rotation2d(-bestTarget.getYaw())), currentVisionTime);
    //                 }
    //             }).until(() -> currentVisionTime - lastVisionTime > Constants.Drivetrain.Vision.TimeoutSeconds).withName("Track target")
    //         )
    //     ).withName("Aim and range")
    //     .finallyDo(() -> {
    //         aimingPoseEstimator = null;
    //     }).withName("Auto aim and range");
    // }
}
