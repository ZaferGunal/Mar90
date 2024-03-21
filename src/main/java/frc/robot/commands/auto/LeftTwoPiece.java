package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
/*import frc.robot.commands.PivotCmd;
import frc.robot.commands.RollerCapture;
import frc.robot.commands.RollerPush;
import frc.robot.commands.ShootCmd;
import frc.robot.subsystems.Intk;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.SH;
import frc.robot.subsystems.SwerveSubsystem;
*/
public class LeftTwoPiece extends SequentialCommandGroup {
   /*public LeftTwoPiece(RollerSubsystem roller, SH shooter, SwerveSubsystem drivebase, Intk intake,
         PathPlannerPath path1, PathPlannerPath path2, PathPlannerPath path3) {
      super(new ShootCmd(shooter).withTimeout(1),
            new ShootCmd(shooter).withTimeout(2).raceWith(new PivotCmd(intake, 5))
                  .raceWith(new RollerPush(roller, 5)),
            AutoBuilder.followPath(path1).raceWith(new PivotCmd(intake, 48)).raceWith(new RollerCapture(roller, 3)),
            AutoBuilder.followPath(path2).raceWith(new PivotCmd(intake, 5)).raceWith(new RollerCapture(roller, 1.3)).raceWith(new ShootCmd(shooter)),
            new ShootCmd(shooter).withTimeout(2).raceWith(new RollerPush(roller, 5)),
            AutoBuilder.followPath(path3));
   }*/
   
}
