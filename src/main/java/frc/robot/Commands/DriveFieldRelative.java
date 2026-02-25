package frc.robot.Commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Subsystems.Drivetrain;

public class DriveFieldRelative extends Command {
    private final Drivetrain swerve;
    private final CommandXboxController controller;
    double translationPow = Constants.Drivetrain.TranslationPow;
    double rotationPow = Constants.Drivetrain.RotationPow;
    /**
     * Creates a new DriveFieldRelative.
     *
     * @param drive The drive subsystem this command wil run on.
     * @param driveController The controller used for running the robot
     */
    public DriveFieldRelative(Drivetrain drive, CommandXboxController driveController) {
        swerve = drive;
        controller = driveController;
        addRequirements(swerve);
    }

    @Override
    public String getName() {
        return "DriveFieldRelative";
    }

    @Override
    public void execute() {
        if(controller.start().getAsBoolean()){
            swerve.resetGyro();
        }

    double x = controller.getLeftX(), y = controller.getLeftY(), theta = controller.getRightX();
    
    if (Math.hypot(x, y) < Constants.controllerDeadband) {
        x = 0;
        y = 0;
    }

    if (Math.abs(theta) < Constants.controllerDeadband) {
        theta = 0.0;
    }

    x = (x > 0) ? Math.abs(Math.pow(x, translationPow)) : -Math.abs(Math.pow(x, translationPow));
    y = (y > 0) ? Math.abs(Math.pow(y, translationPow)) : -Math.abs(Math.pow(y, translationPow));
    theta = (theta > 0) ? Math.abs(Math.pow(theta, rotationPow)) : -Math.abs(Math.pow(theta, rotationPow));

    double slowModeFactor = (controller.getLeftTriggerAxis() * Constants.Drivetrain.SlowFactor) + Constants.Drivetrain.SlowFactorOffset;
    boolean isCosineCompensated = controller.getRightTriggerAxis() >= 0.5;

    swerve.driveFieldRelative(
      new ChassisSpeeds(
        (y * Constants.attainableMaxTranslationalSpeedMPS) / slowModeFactor, 
        (x * Constants.attainableMaxTranslationalSpeedMPS) / slowModeFactor, 
        -(theta * Constants.attainableMaxRotationalVelocityRPS) / slowModeFactor
      ),
      isCosineCompensated
    );
  }
}
