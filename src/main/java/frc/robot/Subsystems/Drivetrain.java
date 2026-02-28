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
import edu.wpi.first.math.geometry.Rotation2d;
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

import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Commands.DriveFieldRelative;

public class Drivetrain extends SubsystemBase {

    // ADIS16470_IMU IMU;
    boolean isCalibrating;
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

    PhotonCamera camera = new PhotonCamera("photonvision");
    SwerveDrivePoseEstimator aimingPoseEstimator;
    double currentVisionTime = 0;
    double lastVisionTime = 0;
    private final PIDController visionForwardBackController = new PIDController(0, 0, 0);
    private final PIDController visionSidewaysController = new PIDController(0, 0, 0);
    private final PIDController visionRotationsController = new PIDController(0, 0, 0);


    private final Alert calibratingAlert = new Alert("Calibrating steering motors", AlertType.kInfo);
    
    public Drivetrain(SwerveModule[] modules, CommandXboxController controller) {
        // IMU = new ADIS16470_IMU();
        IMU = new Pigeon2(Constants.PigeonID, new CANBus("*"));
        this.modules = modules;
        kinematics = new SwerveDriveKinematics(Constants.moduleLocations);
        odometry = new SwerveDriveOdometry(kinematics, getGyroscopeRotation(), getModulePositions());
        statePublisher = NetworkTableInstance.getDefault().getStructArrayTopic("SwerveModules", SwerveModuleState.struct).publish();
        setDefaultCommand(new DriveFieldRelative(this, controller));

        SmartDashboard.putData(prepareToCalibrate());
        SmartDashboard.putData(calibrate());
        SmartDashboard.putData(this);

        
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

    public void resetGyro(){
        // IMU.reset();
    }

    public Command getInitialCommand() {
        if (Preferences.getBoolean("Drivetrain.isPrepartingToCalibrate", true)) {
            return prepareToCalibrate();
        } else {
            return getDefaultCommand();
        }
    }

    public Command prepareToCalibrate() {
        return parallel(
            runOnce(() -> {
                isCalibrating = true;
                Preferences.setBoolean("Drivetrain.isPrepartingToCalibrate", true);
                calibratingAlert.set(true);
            }).andThen(run(() -> {})),
            modules[0].prepareToCalibrate(),
            modules[1].prepareToCalibrate(),
            modules[2].prepareToCalibrate(),
            modules[3].prepareToCalibrate()
        ).withName("Prepare to calibrate");
    }
    
    public Command calibrate() {
        return parallel(
            runOnce(() -> {
                isCalibrating = false;
                Preferences.setBoolean("Drivetrain.isPrepartingToCalibrate", false);
                calibratingAlert.set(false);
            }),
            modules[0].calibrate(),
            modules[1].calibrate(),
            modules[2].calibrate(),
            modules[3].calibrate()
        ).onlyIf(() -> isCalibrating).withName("Calibrate");
    }

    public void drive(ChassisSpeeds  chassisSpeeds){
        setModuleTargetStates(chassisSpeeds, true);
    }

    public void driveFieldRelative(ChassisSpeeds chassisSpeeds, boolean isCosineCompensated){
        chassisSpeeds.vxMetersPerSecond = translationXLimiter.calculate(chassisSpeeds.vxMetersPerSecond);
        chassisSpeeds.vyMetersPerSecond = translationYLimiter.calculate(chassisSpeeds.vyMetersPerSecond);
        chassisSpeeds.omegaRadiansPerSecond = rotationLimiter.calculate(chassisSpeeds.omegaRadiansPerSecond);

        // SmartDashboard.putNumber("gyro", getGyroscopeRotation().getDegrees());

        setModuleTargetStates(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getGyroscopeRotation()), isCosineCompensated);
        // setModuleTargetStates(ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, getGyroscopeRotation()));
    }

    public void setModuleTargetStates(ChassisSpeeds chassisSpeeds, boolean isCosineCompensated) {
        SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(chassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(targetStates, Constants.attainableMaxModuleSpeedMPS);
        for (int i = 0; i < modules.length; ++i) {
            targetStates[i].optimize(Rotation2d.fromRotations(modules[i].getModuleAngRotations()));
            modules[i].setTargetState(targetStates[i], isCosineCompensated);
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
