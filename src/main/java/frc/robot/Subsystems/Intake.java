package frc.robot.Subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    private final SparkFlex roller;

    private double rollerSpeed = 0.5;

    @SuppressWarnings("removal")
    public Intake(int armId, int rollerId) {
        roller = new SparkFlex(rollerId, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(30);

        roller.configure(config, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);

        setDefaultCommand(deploy());
    }

    public Command deploy() {
        // Default command, motion profiled, ideallty feedforward contolled, deploy arm
        // For now we will manually retract
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html
        return run(() -> roller.set(rollerSpeed)).withName("Deploy"); 
    }

    public Command bump() {
        // Motion profiled, ideallty feedforward contolled, raise arm a bit, lower arm after raise
        return runOnce(() -> {});
    }

    // We _might_ need to temporarily slow down intake during shooting but that is to be determined later

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}
