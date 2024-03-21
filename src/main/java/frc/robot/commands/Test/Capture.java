package frc.robot.commands.Test;

import edu.wpi.first.wpilibj2.command.Command;

public class  Capture extends Command


{

   
       

          @Override
          public void initialize() {}
        
          // Called every time the scheduler runs while the command is scheduled.
          @Override
          public void execute() {
           System.out.println(" CAPTURING   CAPTURING  CAPTURING");
        
          }
        
          // Called once the command ends or is interrupted.
          @Override
          public void end(boolean interrupted) {
           System.out.println("capturing ENDed   CAPturing Ended \n CAPTURING ENDED    CAPTURING ENDED \n   CAPTURING ENDED CAPTURİNG ENDED ");
          }
        
          // Returns true when the command should end.
          @Override
          public boolean isFinished() {
            return false;
          }



}
