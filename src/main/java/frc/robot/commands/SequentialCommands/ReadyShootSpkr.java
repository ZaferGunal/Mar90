package frc.robot.commands.SequentialCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SytsemCommands.RollerCommands.RollerPush;
import frc.robot.commands.SytsemCommands.ShooterCommands.ShootAmp;
import frc.robot.commands.SytsemCommands.ShooterCommands.ShootSpkr;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.ShooterSystem;
public class ReadyShootSpkr extends SequentialCommandGroup{

    public ReadyShootSpkr(IntakeSystem intakeSystem,ShooterSystem shooterSystem){
        super(new ShootSpkr(shooterSystem).withTimeout(2),(new ShootSpkr(shooterSystem).alongWith(new RollerPush(intakeSystem).withTimeout(2))));

    }
    

}
