// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;  
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.commands.PivotBackwardCmd;

import frc.robot.commands.ShootCmd;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Translation2d;

  import frc.robot.subsystems.ShooterSystem;



public class RobotContainer {
  SwerveSubsystem drivebase = new SwerveSubsystem();
  IntakeSystem intakeSystem = new IntakeSystem();
    ShooterSystem shooterSystem = new ShooterSystem();
  
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
   /*  new JoystickButton(driverXbox,3).whileTrue(new ShootCmd(shooterSystem));
    new JoystickButton(driverXbox,2).whileTrue(new PivotBackwardCmd(intakeSystem,8));
    new JoystickButton(driverXbox,1).whileTrue(new PivotBackwardCmd(intakeSystem,42));*/
    new JoystickButton(driverXbox,1).whileTrue(drivebase.driveToPose(new Pose2d(new Translation2d(0,0),new Rotation2d(Math.PI/2))));
  }

  public Command getAutonomousCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Path1");
    drivebase.resetOdometry(path.getPreviewStartingHolonomicPose());
    return AutoBuilder.followPath(path);

  }
}
