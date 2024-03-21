package frc.robot.commands.SequentialCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.commands.SytsemCommands.PivotCommands.Pivot2Feed;
import frc.robot.commands.SytsemCommands.PivotCommands.Pivot2Floor;
import frc.robot.commands.SytsemCommands.RollerCommands.RollerCapture;
public class ReadyCapture extends SequentialCommandGroup {
    
    public ReadyCapture(IntakeSystem intakeSystem, ShooterSystem shooterSystem){
        super(new Pivot2Floor(intakeSystem).alongWith(new RollerCapture(intakeSystem)));
    }
   


}
