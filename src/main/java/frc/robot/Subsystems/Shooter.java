package frc.robot.Subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants;


public class Shooter extends SubsystemBase {
    private final SparkFlex shooter;
    private final RelativeEncoder encoder;

    private double primeRPM = 0;
    private double shootRPM = 0;

    final PIDController controller = new PIDController(Constants.Shooter.kP, Constants.Shooter.kI, Constants.Shooter.kD);
    final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(Constants.Shooter.kS, Constants.Shooter.kV);
    
    @SuppressWarnings("removal")
    public Shooter(int MotorID) {
        shooter = new SparkFlex(MotorID, MotorType.kBrushless);
        encoder = shooter.getEncoder();
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kBrake).smartCurrentLimit(30);

        shooter.configure(config, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);

        setDefaultCommand(prime());
    }

    // _If_ we need a third state where motor does not turn, turn off the motor
    // when leaving the other two states and have no default command.


    public void setSpeed(double currentRPM, double targetRPM) {
        double pidOutput = controller.calculate(currentRPM, targetRPM);
        double ffOutput = ff.calculate(targetRPM);
        shooter.setVoltage(pidOutput + ffOutput);
    }

    public Command prime() {
        // Default command, rotate slowly to reduce shooting prep time
        double currentRPM = encoder.getVelocity();
        return runEnd(() -> setSpeed(currentRPM, primeRPM), () -> {}).withName("Prime");
    }

    public Command shoot() {
        // Speed up rollers, control velocty
        // https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-flywheel.html
        double currentRPM = encoder.getVelocity();
        return runEnd(() -> setSpeed(currentRPM, shootRPM), () -> {}).withName("Shoot");
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}
