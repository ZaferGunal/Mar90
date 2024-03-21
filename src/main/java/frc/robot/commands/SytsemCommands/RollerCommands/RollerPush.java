package frc.robot.commands.SytsemCommands.RollerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSystem;
public class RollerPush  extends Command{
    
    IntakeSystem intakeSystem ;


    public RollerPush(IntakeSystem intakeSystem_){
        intakeSystem = intakeSystem_;
    }



 
    @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSystem.rollerPush();


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSystem.stopRollerMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  


}    
}
