package frc.robot.Subsystems;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Intake extends SubsystemBase {
    private final SparkFlex roller;

    @SuppressWarnings("removal")
    public Intake(int rollerId) {
        roller = new SparkFlex(rollerId, MotorType.kBrushless);
        SparkFlexConfig rollerConfig = new SparkFlexConfig();
        rollerConfig.inverted(true);
        rollerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(30);

        roller.configure(rollerConfig, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);
        setDefaultCommand(intake());
    }


    public Command intake() {
        return runEnd(() -> {
            roller.setVoltage(Constants.Intake.rollerVoltage);
        }, () -> {}).withName("Intake");
        // return run(() -> {});
    }

    public Command discharge() {
        return runEnd(() -> {
            roller.setVoltage(-Constants.Intake.rollerVoltage);
        }, () -> {}).withName("Intake");
        // return run(() -> {});
    }

    public Command stop() {
        return runEnd(() -> {
            roller.setVoltage(0);
        }, () -> {}).withName("Stop intake");
        // return run(() -> {});
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}