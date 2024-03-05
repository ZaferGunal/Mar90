// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.PivotBackwardCmd;
import frc.robot.commands.PivotForwardCmd;


public class RobotContainer {
  private final SwerveSubsystem drivebase = new SwerveSubsystem();
 IntakeSystem intakeSystem = new IntakeSystem();
  
  final Joystick driverXbox = new Joystick(0);
  
  public RobotContainer() {
    configureBindings();
               
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> MathUtil.applyDeadband(driverXbox.getRawAxis(1), 0.5)*-1,
        () -> MathUtil.applyDeadband(driverXbox.getRawAxis(0), 0.5)*-1,
        () -> driverXbox.getRawAxis(4) * -0.5);
    drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
  }

  private void configureBindings() {
    
    new JoystickButton(driverXbox,2).whileTrue(new PivotBackwardCmd(intakeSystem));
    new JoystickButton(driverXbox,1).whileTrue(new PivotForwardCmd(intakeSystem));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");

  }
}
