package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.RollerSubsystem;
public class RollerPush  extends Command{

RollerSubsystem intakeSystem;
 double voltage ;
    public RollerPush(RollerSubsystem intakeSystem_, double absoluteVoltage_){
        intakeSystem = intakeSystem_;
      voltage = absoluteVoltage_;
    }

    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        intakeSystem.setRollerSpeed(-1 * voltage);
    
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      intakeSystem.stopRoller();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
}
    
}