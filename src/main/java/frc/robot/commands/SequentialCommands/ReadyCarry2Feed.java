package frc.robot.commands.SequentialCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.commands.SytsemCommands.RollerCommands.RollerHold;
import frc.robot.commands.SytsemCommands.PivotCommands.Pivot2Feed;
public class ReadyCarry2Feed extends SequentialCommandGroup{
 
    public ReadyCarry2Feed(IntakeSystem intakeSystem ) {
        super(new RollerHold(intakeSystem).alongWith(new Pivot2Feed(intakeSystem)));
    };
    
    

    
}
