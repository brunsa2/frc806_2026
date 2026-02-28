import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    public Intake(int ArmID, int RollerID) {

    }

    public Command deploy() {
        // Default command, motion profiled, ideallty feedforward contolled, deploy arm
        // Turn on roller (no control needed)
        // For now we will manually retract
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-vertical-arm.html
        return runOnce(() -> {}); 
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
