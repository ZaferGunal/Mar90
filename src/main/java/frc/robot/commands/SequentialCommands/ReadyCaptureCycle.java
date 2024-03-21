package frc.robot.commands.SequentialCommands;

import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.commands.SytsemCommands.PivotCommands.Pivot2Feed;
import frc.robot.commands.SytsemCommands.PivotCommands.Pivot2Floor;
import frc.robot.commands.SequentialCommands.ReadyCarry2Feed;
import frc.robot.commands.SytsemCommands.RollerCommands.RollerCapture;
public class ReadyCaptureCycle extends SequentialCommandGroup {
    
    IntakeSystem intakeSystem ;
    public ReadyCaptureCycle(IntakeSystem intakeSystem_, ShooterSystem shooterSystem_){
        super(new Pivot2Floor(intakeSystem_).alongWith(new RollerCapture(intakeSystem_)));
        intakeSystem = intakeSystem_;
    }
      
    public void endThen(){
        new ReadyCarry2Feed(intakeSystem).schedule();
    }

   

    

}
