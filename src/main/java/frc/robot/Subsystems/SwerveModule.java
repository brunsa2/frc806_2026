package frc.robot.Subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase{

    //drive 
    TalonFX driveMotor;
    int driveMotorID;
    int encoderID;
    //steer
    TalonFX steerMotor;
    PIDController steerController;
    //module encoder 
    CANcoder moduleEncoder;
    private static final String EncoderPreferenceKey = "EncoderOffset";
    //conversion factors
    final double WHEEL_DIAMETER = Units.inchesToMeters(4);
    final double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER * Math.PI;
    final double GEAR_RATIO = 1.0 / 5.27;
    final double DRIVE_POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
    final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION / 60.0;
    final double STEER_POSITION_CONVERSION = 1;
    final double STEER_VELOCITY_CONVERSION = STEER_POSITION_CONVERSION / 60.0;
    private final SlewRateLimiter steerLimiter = new SlewRateLimiter(Constants.Drivetrain.SteerMotorSlewRate);

    final Alert SwerveDriveMotorAlert;
    final Alert SwerveSteerMotorAlert;
    final Alert SwerveEncoderAlert;
    final String SwerveRotationName;

    public SwerveModule(int driveMotorID, int steerMotorID, int encoderID){
        this.driveMotorID = driveMotorID;
        this.encoderID = encoderID;

        SwerveDriveMotorAlert = new Alert("Lost module " + encoderID + " drive motor(" + driveMotorID + ")", AlertType.kError);
        SwerveSteerMotorAlert = new Alert("Lost module " + encoderID + " steer motor(" + steerMotorID + ")", AlertType.kError);
        SwerveEncoderAlert = new Alert("Lost module " + encoderID + " encoder(" + encoderID + ")", AlertType.kError);
        SwerveRotationName = "Swerve " + encoderID + " rotation";

        //drive motor 
        driveMotor = new TalonFX(driveMotorID);
        var driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.Drivetrain.DriveMotorsHighSupplyCurrentLimit;
        driveMotorConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.Drivetrain.DriveMotorsLowSupplyCurrentLimit;
        driveMotorConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.Drivetrain.DriveMotorsHighSupplyCurrentSeconds;
        driveMotorConfig.Feedback.SensorToMechanismRatio = 1/DRIVE_POSITION_CONVERSION;
        driveMotorConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        driveMotor.getConfigurator().apply(driveMotorConfig);
        driveMotor.setPosition(0);

        //steer motor
        steerMotor = new TalonFX(steerMotorID);
        var steerMotorConfig = new TalonFXConfiguration();
        steerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerMotorConfig.CurrentLimits.SupplyCurrentLimit = Constants.Drivetrain.SteerMotorsSupplyCurrentLimit;
        steerMotor.getConfigurator().apply(steerMotorConfig);
        
        // module encoder
        moduleEncoder = new CANcoder(encoderID, new CANBus("*"));
        var steerEncoderConfig = new CANcoderConfiguration();
        steerEncoderConfig.MagnetSensor.MagnetOffset = -Preferences.getDouble(EncoderPreferenceKey + encoderID, 0);
        steerEncoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        moduleEncoder.getConfigurator().apply(steerEncoderConfig);

        //controllers
        //driveController = driveMotor.getPIDController();
        //driveController.setP(Constants.Modules.SpeedKP);
        //driveController.setI(Constants.Modules.SpeedKI);
        //driveController.setD(Constants.Modules.SpeedKD);

        steerController = new PIDController(Constants.Drivetrain.SteerDriveKP, Constants.Drivetrain.SteerDriveKI, Constants.Drivetrain.SteerDriveKD);
        steerController.enableContinuousInput(-0.5, 0.5);
    }

    private SlewRateLimiter srl = new SlewRateLimiter(0.8);

    public void setTargetState(SwerveModuleState targetState) {
        //PID experement
        //steerMotor.set(-steerController.calculate(getModuleAngRotations(),targetState.angle.getRotations()));
        //driveController.setReference(targetState.speedMetersPerSecond / DRIVE_VELOCITY_CONVERSION, ControlType.kVelocity);

        double currentAngle = getModuleAngRotations();
        // TODO: steer PID on motor controller with external cancoder sensor
        double steerMotorCommand = steerController.calculate(currentAngle, targetState.angle.getRotations());
        steerMotor.set(steerLimiter.calculate(steerMotorCommand));
        // Cosine compensation: drive wheel slower when it's not rotated to the correct position yet
        targetState.speedMetersPerSecond *= targetState.angle.minus(new Rotation2d(currentAngle*2*Math.PI)).getCos();
        var clampedSpeed = MathUtil.clamp(targetState.speedMetersPerSecond/Constants.attainableMaxModuleSpeedMPS, -0.1, 0.1);
        var slewedSpeed = srl.calculate(clampedSpeed);
        driveMotor.set(slewedSpeed); 
    }

    public Command calibrate() {
        return runOnce(() -> {
            var adjustedEncoderValue = moduleEncoder.getAbsolutePosition().getValueAsDouble();
            var steerEncoderConfig = new CANcoderConfiguration();
            moduleEncoder.getConfigurator().refresh(steerEncoderConfig);
            double offset = steerEncoderConfig.MagnetSensor.MagnetOffset;
            var rawEncoderValue = adjustedEncoderValue - offset;
            steerEncoderConfig.MagnetSensor.MagnetOffset = -rawEncoderValue;
            moduleEncoder.getConfigurator().apply(steerEncoderConfig);
            Preferences.setDouble(EncoderPreferenceKey + encoderID, rawEncoderValue);
            if (Math.abs(moduleEncoder.getAbsolutePosition().waitForUpdate(0.1).getValueAsDouble()) < 0.01) {
                System.out.println("Succesfully calibrated swerve " + encoderID);
            }
            else {
                System.out.println("Swerve " + encoderID + " calibration failed");
            }
        }).ignoringDisable(true).withName("Calibrate");
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("S" + driveMotorID, driveMotor.getPosition().getValueAsDouble());
        setAlerts();
        SmartDashboard.putNumber(SwerveRotationName, getModuleAngRotations());
    }

    public void setAlerts() {
        var deviceStatus = getDeviceStatus();
        SwerveDriveMotorAlert.set(!deviceStatus[0]);
        SwerveSteerMotorAlert.set(!deviceStatus[1]);
        SwerveEncoderAlert.set(!deviceStatus[2]);
    }

    public boolean getSwerveOperational() {
        return driveMotor.isConnected() && steerMotor.isConnected() && moduleEncoder.isConnected();
    }

    public boolean[] getDeviceStatus() {
        return new boolean[]{driveMotor.isConnected(), steerMotor.isConnected(), moduleEncoder.isConnected()};
    }
    

    public double getModuleAngRotations() {
        return moduleEncoder.getAbsolutePosition().getValueAsDouble();
    }

    public SwerveModulePosition getModulePosition() {
        return new SwerveModulePosition(
            driveMotor.getPosition().getValueAsDouble(),
            Rotation2d.fromRotations(getModuleAngRotations())
        );  
    }

    public SwerveModuleState getSwerveModuleState() {
        return new SwerveModuleState(
            driveMotor.getVelocity().getValueAsDouble(),
            Rotation2d.fromRotations(getModuleAngRotations()));
    }
}
