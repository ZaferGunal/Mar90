package frc.robot.commands.auto;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.PivotCmd;
import frc.robot.commands.RollerCapture;
import frc.robot.commands.RollerPush;
import frc.robot.commands.ShootCmd;
import frc.robot.subsystems.IntakeSystem;
import frc.robot.subsystems.RollerSubsystem;
import frc.robot.subsystems.ShooterSystem;
import frc.robot.subsystems.SwerveSubsystem;




public class ThreePiece extends SequentialCommandGroup {
   public ThreePiece(RollerSubsystem roller, ShooterSystem shooter, SwerveSubsystem drivebase, IntakeSystem intake,
         PathPlannerPath path1, PathPlannerPath path2, PathPlannerPath path3, PathPlannerPath path4,
         PathPlannerPath path5) {
      super(new ShootCmd(shooter).withTimeout(1),
            new ShootCmd(shooter).withTimeout(2).raceWith(new PivotCmd(intake, 5))
                  .raceWith(new RollerPush(roller, 5)),
            AutoBuilder.followPath(path1).raceWith(new PivotCmd(intake, 48)).raceWith(new RollerCapture(roller, 3)),
            AutoBuilder.followPath(path2).raceWith(new PivotCmd(intake, 5)).raceWith(new RollerCapture(roller, 1.3)).raceWith(new ShootCmd(shooter)),
            new ShootCmd(shooter).withTimeout(2).raceWith(new RollerPush(roller, 5)),
            AutoBuilder.followPath(path3).raceWith(new PivotCmd(intake, 48)).raceWith(new RollerCapture(roller, 3)),
            AutoBuilder.followPath(path4).raceWith(new PivotCmd(intake, 5)).raceWith(new RollerCapture(roller, 1.3)).raceWith(new ShootCmd(shooter)),
                  new ShootCmd(shooter).withTimeout(2).raceWith(new RollerPush(roller, 5)),
                        AutoBuilder.followPath(path5));
   }
}
