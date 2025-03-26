// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.Spit;
import frc.robot.commands.TeleopDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Roller;
import frc.robot.subsystems.Arm;

public class RobotContainer {
  private final CommandXboxController controller = new CommandXboxController(Constants.driveControllerId);

  private final Drivetrain drivetrain = new Drivetrain();
  private final Roller roller = new Roller();
  private final Arm arm = new Arm();
  
  private final LoggedDashboardChooser<Command> autoChooser;

  public RobotContainer() {
    autoChooser = new LoggedDashboardChooser<>("Auto Choices", AutoBuilder.buildAutoChooser());

    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(new TeleopDrive(
      drivetrain,
      () -> -controller.getLeftY(),
      () -> -controller.getRightX()
    ));

    controller.leftBumper().whileTrue(new Spit(roller));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
