package frc.robot.commands.SytsemCommands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;
public class ShootAmp  extends Command{
    
    ShooterSystem  shooterSystem ;


    public ShootAmp(ShooterSystem shooterSystem_){
        shooterSystem = shooterSystem_;
    }



 
    @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
     shooterSystem.shoot2amp();



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSystem.stopShooter();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  


}    
}
