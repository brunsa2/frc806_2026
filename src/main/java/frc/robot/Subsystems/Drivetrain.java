package frc.robot.Subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
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
import static edu.wpi.first.wpilibj2.command.Commands.parallel;
import static edu.wpi.first.wpilibj2.command.Commands.waitSeconds;
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

        SmartDashboard.putData(enableCalibration());
        SmartDashboard.putData(calibrate());
        SmartDashboard.putData(cancelCalibration());
        SmartDashboard.putData(this);

        
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
}
