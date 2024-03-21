package frc.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.Command;

public class Hold extends Command


{

   
       

          @Override
          public void initialize() {}
        
          // Called every time the scheduler runs while the command is scheduled.
          @Override
          public void execute() {
           System.out.println(" HOLDING    HOLDING   HOLDING  ");
        
          }
        
          // Called once the command ends or is interrupted.
          @Override
          public void end(boolean interrupted) {
           System.out.println("HOLDING ENDed   HOLDING Ended \n CAPTURING ENDED    HOLDING ENDED \n   HOLDING  ENDED HOLFING ENDED ");
          }
        
          // Returns true when the command should end.
          @Override
          public boolean isFinished() {
            return false;
          }



}
