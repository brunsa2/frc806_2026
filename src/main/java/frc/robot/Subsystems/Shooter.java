import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Shooter extends SubsystemBase {
    private final SparkFlex shooter;
    
    public Shooter(int MotorID) {
        shooter = new SparkFlex(MotorID, MotorType.kBrushless);

        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kCoast).smartCurrentLimit(30);

        shooter.configure(config, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);
    }

    // _If_ we need a third state where motor does not turn, turn off the motor
    // when leaving the other two states and have no default command.

    public Command prime() {
        // Default command, rotate slowly to reduce shooting prep time
        return runOnce(() -> shooter.set(0.1)).withName("Prime");
    }

    public Command shoot() {
        // Speed up rollers, control velocty
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html
        return runOnce(() -> shooter.set(0.9)).withName("Shoot");
    }

    @Override
    public void initSendable(SendableBuilder builder) {

    }
}
