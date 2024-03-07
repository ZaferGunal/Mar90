package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSystem;
public class ShootCmd extends Command {

    ShooterSystem shooterSystem ;

public ShootCmd(ShooterSystem shooterSystem_){
shooterSystem = shooterSystem_;
}

    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        shooterSystem.shoot();
    
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      shooterSystem.stopMotors();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
}
}