package frc.robot.commands.auto;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootCmd;
import frc.robot.subsystems.ShooterSystem;

public class ShootAndWait extends SequentialCommandGroup{
    public ShootAndWait(ShooterSystem shooter){
    super(new ShootCmd(shooter).withTimeout(2));
    }

}
