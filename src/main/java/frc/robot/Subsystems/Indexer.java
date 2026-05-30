package frc.robot.Subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;

public class Indexer extends SubsystemBase {
    private final SparkFlex topRoller;

    @SuppressWarnings("removal")
    public Indexer(int topRollerID) {
        topRoller = new SparkFlex(topRollerID, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(30);
        config.inverted(true);

        topRoller.configure(config, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);

        setDefaultCommand(idleIndex());
    }

    public Command idleIndex() {
        return runEnd(() -> {
            topRoller.setVoltage(-Constants.Indexer.ceilingIdleVoltage);
        }, () -> {}).withName("Idle index");
        // return run(() -> {});
    }

    public Command index() {
        return runEnd(() -> {
            topRoller.setVoltage(Constants.Indexer.ceilingIndexVoltage);
        }, () -> {}).withName("Index");
        // return run(() -> {});
    }

    public Command stop() {
        return runEnd(() -> {
            topRoller.setVoltage(0);
        }, () -> {}).withName("Stop indexer");
        // return run(() -> {});
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}
