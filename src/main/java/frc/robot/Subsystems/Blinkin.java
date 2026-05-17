package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

import frc.robot.Constants;

public class Blinkin extends SubsystemBase {
    private final PWMSparkMax blinkin;
    private final Constants.Blinkin.SolidColors defaultColor;

    public Blinkin(int PWMslot, Constants.Blinkin.SolidColors defaultColor) {
        this.defaultColor = defaultColor;
        blinkin = new PWMSparkMax(PWMslot);
        setDefaultCommand(setDefaultColor());
    }

    public Command setDefaultColor() {
        return runOnce(() -> blinkin.set(defaultColor.value));
    }

    public Command setColor(Constants.Blinkin.SolidColors color) {
        return runOnce(() -> blinkin.set(color.value));
    }

    public Command turnOffLEDs() {
        return runOnce(() -> blinkin.set(Constants.Blinkin.SolidColors.BLACK.value));
    }
}
