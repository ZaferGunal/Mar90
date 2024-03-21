package frc.robot.commands.SytsemCommands.PivotCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSystem;
public class Pivot2Floor  extends Command{
    
    IntakeSystem intakeSystem ;


    public Pivot2Floor(IntakeSystem intakeSystem_){
        intakeSystem = intakeSystem_;
    }



 
    @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakeSystem.setPivot2floor();


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSystem.stopPivotMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  


}    
}
