// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Subsystems.Pose;
import frc.robot.Subsystems.Drivetrain;
import frc.robot.Subsystems.Indexer;
import frc.robot.Subsystems.Shooter;
import frc.robot.Subsystems.Intake;

public class RobotContainer {
    public CommandXboxController driveController = new CommandXboxController(0);
    CommandXboxController coDriveController = new CommandXboxController(1);
    CommandXboxController ohShitController = new CommandXboxController(2);

    private final Trigger driveRightTrigger = driveController.rightTrigger(0.5);
    private final Trigger drivekLeftBumper = driveController.leftBumper();
    private final Trigger drivekRightBumper = driveController.rightBumper();

    public final Drivetrain drivetrain = new Drivetrain(Constants.Drivetrain.moduleArray, driveController);
    public final Pose pose = new Pose(Constants.Pose.CameraName, drivetrain::getKinematics, drivetrain::getGyroscopeRotation, drivetrain::getModulePositions);
    
    public final Indexer indexer = new Indexer(Constants.Indexer.BottomRollerID, Constants.Indexer.TopRollerID);
    public final Shooter shooter = new Shooter(Constants.Shooter.MotorID);
    public final Intake intake = new Intake(Constants.Intake.ArmID, Constants.Intake.RollerID);

    public RobotContainer() {
        drivetrain.setPose(pose);

        configureBindings();
        SmartDashboard.putData(CommandScheduler.getInstance());
    }

    private void configureBindings() {
        // driveRightTrigger.whileTrue(parallel(indexer.index(), shooter.shoot()));
        // drivekRightBumper.toggleOnTrue(intake.top());
        // drivekLeftBumper.whileTrue(intake.bump());
        driveRightTrigger.whileTrue(drivetrain.alignToTag(7, new Translation2d(-1, -0.5), Rotation2d.fromDegrees(-135)));
    }
}
