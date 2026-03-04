package frc.robot.Subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Indexer extends SubsystemBase {
    private final SparkFlex bottomRoller;
    private final SparkFlex topRoller;

    private final double floorIdleSpeed = 0.1;
    private final double floorIndexSpeed = 0.4;
    private final double ceilingIdleSpeed = 0.2;
    private final double ceilingIndexSpeed = 0.7;

    @SuppressWarnings("removal")
    public Indexer(int bottomRollerID, int topRollerID) {
        bottomRoller = new SparkFlex(bottomRollerID, MotorType.kBrushless);
        topRoller = new SparkFlex(topRollerID, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(30);
        config.inverted(true);

        bottomRoller.configure(config, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);
        topRoller.configure(config, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);

        setDefaultCommand(idleIndex());
    }

    public Command idleIndex() {
        return runEnd(() -> {
            bottomRoller.set(floorIdleSpeed);
            topRoller.set(-ceilingIdleSpeed);
        }, () -> {}).withName("Idle index");
    }

    public Command index() {
        return runEnd(() -> {
            bottomRoller.set(floorIndexSpeed);
            topRoller.set(ceilingIndexSpeed);
        }, () -> {}).withName("Index");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}
