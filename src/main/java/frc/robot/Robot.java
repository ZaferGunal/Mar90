// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autoCommand ;
 private RobotContainer robotContainer;
 SendableChooser<String> autoChooser= new SendableChooser() ;


  @Override
  public void robotInit() {
   // autoChooser.addOption("Mid / 2 Obj from Left", "mid / 2 Obj from left");
      SmartDashboard.putData("AutoChooser",autoChooser);
      robotContainer = new RobotContainer();
  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
   
    m_autoCommand = robotContainer.getAutonomousCommand(autoChooser.getSelected());
    if(m_autoCommand != null){
      m_autoCommand.schedule();
    }
    
    }

   
  

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
   if(m_autoCommand != null){
    m_autoCommand.cancel();
   }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
