package frc.robot.Subsystems;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants;


public class Shooter extends SubsystemBase {
    private final SparkFlex shooter;
    private final RelativeEncoder encoder;

    private double shootRPS = Constants.Shooter.ShootRPM / 60.0;

    private final PIDController controller = new PIDController(Constants.Shooter.kP, Constants.Shooter.kI, Constants.Shooter.kD);
    private final SimpleMotorFeedforward ff = new SimpleMotorFeedforward(Constants.Shooter.kS, Constants.Shooter.kV);
    
    private final Alert shooterGoodAlert = new Alert("Shooter at target speed!", AlertType.kInfo);
    private final Alert shooterBadAlert = new Alert("Shooter not at target speed!", AlertType.kInfo);

    @SuppressWarnings("removal")
    public Shooter(int MotorID) {
        shooter = new SparkFlex(MotorID, MotorType.kBrushless);
        encoder = shooter.getEncoder();
        SparkFlexConfig config = new SparkFlexConfig();
        config.idleMode(IdleMode.kCoast).smartCurrentLimit(30);
        config.inverted(true);
        config.encoder.velocityConversionFactor(1.0 / 60.0);
        shooter.configure(config, SparkFlex.ResetMode.kResetSafeParameters, SparkFlex.PersistMode.kPersistParameters);

        setDefaultCommand(shoot());
    }

    // _If_ we need a third state where motor does not turn, turn off the motor
    // when leaving the other two states and have no default command.


    public void setSpeed(double targetRPS) {
        double currentRPS = encoder.getVelocity();
        double pidOutput = controller.calculate(currentRPS, targetRPS);
        double ffOutput = ff.calculate(targetRPS);
        shooter.setVoltage(pidOutput + ffOutput);
    }

    public boolean check_correct_speed(double targetRPS, double currentRPS) {
        if (targetRPS <= 0) return false; 
        double error = Math.abs(targetRPS - currentRPS);
        return (error / targetRPS) < 0.03;
    }

    public void speed_alert(boolean status) {
        if (status) {
            shooterGoodAlert.set(true);
            shooterBadAlert.set(false);
        }
        else {
            shooterBadAlert.set(true);
            shooterGoodAlert.set(false);
        }
    }

    public void shooter_debug() {
        double currentRPS = encoder.getVelocity();
        double target = shootRPS;
        SmartDashboard.putNumber("Shooter RPS", currentRPS);
        SmartDashboard.putNumber("Shooter RPM", currentRPS * 60.0);
        SmartDashboard.putNumber("Shooter error", target - currentRPS);
        speed_alert(check_correct_speed(target, currentRPS));
    }

    public Command shoot() {
        return runEnd(() -> {
            setSpeed(shootRPS);
        }, () -> {}).withName("Shoot");
        // return run(() -> {});
    }

    public Command stop() {
        return runEnd(() -> {
            shooter.setVoltage(0);
        }, () -> {}).withName("Stop shooter");
        // return run(() -> {});
    }

    @Override
    public void periodic() {
        shooter_debug();
    }

    @Override
    public void initSendable(SendableBuilder builder) {
    }
}
