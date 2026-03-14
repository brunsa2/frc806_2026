package frc.robot.Subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final SparkFlex roller;
    private final TalonFX arm;
    private final MotionMagicExpoVoltage armRequest;

    private double rollerSpeed = 0.5;

    @SuppressWarnings("removal")
    public Intake(int armId, int rollerId) {
        roller = new SparkFlex(rollerId, MotorType.kBrushless);
        SparkFlexConfig rollerConfig = new SparkFlexConfig();
        rollerConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(30);

        roller.configure(rollerConfig, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);


        arm = new TalonFX(armId);
        TalonFXConfiguration armConfig = new TalonFXConfiguration();

        armConfig.Feedback.FeedbackRemoteSensorID = Constants.Intake.ArmEncoderId;
        armConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        armConfig.Feedback.SensorToMechanismRatio = 1.0;

        armConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        armConfig.CurrentLimits.SupplyCurrentLimit = 40;

        var s0 = armConfig.Slot0;
        s0.GravityType = GravityTypeValue.Arm_Cosine;
        s0.kG = Constants.Intake.kG;
        s0.kS = Constants.Intake.kS;
        s0.kV = Constants.Intake.kV;
        s0.kA = Constants.Intake.kA;
        s0.kP = Constants.Intake.kP;
        s0.kI = Constants.Intake.kI;
        s0.kD = Constants.Intake.kD;

        var motionMagicConfigs = armConfig.MotionMagic;
        motionMagicConfigs.MotionMagicCruiseVelocity = Constants.Intake.MotionMagicCruiseVelocity;
        motionMagicConfigs.MotionMagicExpo_kV = Constants.Intake.kV;
        motionMagicConfigs.MotionMagicExpo_kA = Constants.Intake.kA;

        double ForwardDegreeLimit = 90.0;
        double BackwardDegreeLimit = 90.0;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ForwardDegreeLimit/360.0;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        armConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = -BackwardDegreeLimit/360.0; 
        armConfig.ClosedLoopGeneral.ContinuousWrap = false;
        armConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        arm.getConfigurator().apply(armConfig);

        armRequest = new MotionMagicExpoVoltage(0);

        setDefaultCommand(deploy());
    }

    public Command deploy() {
        // Default command, motion profiled, ideallty feedforward contolled, deploy arm
        // For now we will manually retract
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html
        //arm.setControl(armRequest.withPosition(0));
        return run(() -> roller.set(rollerSpeed)).withName("Deploy"); 
    }

    public Command bump() {
        // Motion profiled, ideallty feedforward contolled, raise arm a bit, lower arm after raise
        //arm.setControl(armRequest.withPosition(0));
        return runOnce(() -> {});
    }

    // We _might_ need to temporarily slow down intake during shooting but that is to be determined later

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}