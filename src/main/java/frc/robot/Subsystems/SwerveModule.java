package frc.robot.Subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Preferences;
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
    final double GEAR_RATIO = 1.0 / 6.75;
    final double DRIVE_POSITION_CONVERSION = WHEEL_CIRCUMFERENCE * GEAR_RATIO;
    final double DRIVE_VELOCITY_CONVERSION = DRIVE_POSITION_CONVERSION / 60.0;
    final double STEER_POSITION_CONVERSION = 1;
    final double STEER_VELOCITY_CONVERSION = STEER_POSITION_CONVERSION / 60.0;
    private final SlewRateLimiter steerLimiter = new SlewRateLimiter(Constants.Drivetrain.SteerMotorSlewRate);

    public SwerveModule(int driveMotorID, int steerMotorID, int encoderID, boolean invertDirection){
        this.driveMotorID = driveMotorID;
        this.encoderID = encoderID;

        //drive motor 
        driveMotor = new TalonFX(driveMotorID);
        var driveMotorConfig = new TalonFXConfiguration();
        driveMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveMotorConfig.CurrentLimits.SupplyCurrentLimit = 40;
        if (invertDirection) {
            driveMotorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        }
        driveMotor.getConfigurator().apply(driveMotorConfig);

        //steer motor
        steerMotor = new TalonFX(steerMotorID);
        var steerMotorConfig = new TalonFXConfiguration();
        steerMotorConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerMotorConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerMotorConfig.CurrentLimits.SupplyCurrentLimit = 20;
        steerMotor.getConfigurator().apply(steerMotorConfig);
        
        // module encoder
        moduleEncoder = new CANcoder(encoderID, new CANBus("*"));

        //controllers
        //driveController = driveMotor.getPIDController();
        //driveController.setP(Constants.Modules.SpeedKP);
        //driveController.setI(Constants.Modules.SpeedKI);
        //driveController.setD(Constants.Modules.SpeedKD);

        steerController = new PIDController(Constants.Modules.SteerKP, Constants.Modules.SteerKI, Constants.Modules.SteerKD);
        steerController.enableContinuousInput(0 - Preferences.getDouble(EncoderPreferenceKey + encoderID, 0), 1 - Preferences.getDouble(EncoderPreferenceKey + encoderID, 0));

    }

    public void setTargetState(SwerveModuleState targetState, boolean isCosineCompensated) {
        //PID experement
        //steerMotor.set(-steerController.calculate(getModuleAngRotations(),targetState.angle.getRotations()));
        //driveController.setReference(targetState.speedMetersPerSecond / DRIVE_VELOCITY_CONVERSION, ControlType.kVelocity);

        double currentAngle = getModuleAngRotations();
        double steerMotorCommand = steerController.calculate(currentAngle, targetState.angle.getRotations());
        steerMotor.set(steerLimiter.calculate(steerMotorCommand));
        if (isCosineCompensated) {
            targetState.speedMetersPerSecond *= targetState.angle.minus(new Rotation2d(currentAngle*2*Math.PI)).getCos();
        }
        driveMotor.set(targetState.speedMetersPerSecond/Constants.attainableMaxModuleSpeedMPS); 
    }

    public Command calibrate() {
        return runOnce(() -> {
            var encoderValue = moduleEncoder.getAbsolutePosition().getValueAsDouble();
            Preferences.setDouble(EncoderPreferenceKey + encoderID, encoderValue);
        }).withName("Calibrate");
    }

    public void periodic() {
        // SmartDashboard.putNumber("S" + driveMotorID, getModuleAngRotations());
    }

    public double getModuleAngRotations() {
        return moduleEncoder.getAbsolutePosition().getValueAsDouble() - Preferences.getDouble(EncoderPreferenceKey + encoderID, 0);
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
