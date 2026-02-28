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
    public Indexer(int BottomRollerID, int TopRollerID) {
        bottomRoller = new SparkFlex(BottomRollerID, MotorType.kBrushless);
        topRoller = new SparkFlex(TopRollerID, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kCoast).smartCurrentLimit(30);

        bottomRoller.configure(config, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);
        topRoller.configure(config, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);
    }

    // No default command

    public Command index() {
        // Spin motors at correct duty cycles. We should not need controllers.
        return runOnce(() -> {
            bottomRoller.set(0.4);
            topRoller.set(0.4);
        }).withName("Start Indexing");
    }

    @Override
    public void initSendable(SendableBuilder builder) {

    }
}
