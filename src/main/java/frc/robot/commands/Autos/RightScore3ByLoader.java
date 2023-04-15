package frc.robot.commands.Autos;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmToHome;
import frc.robot.commands.IntakeCone;
import frc.robot.commands.IntakeCube;
import frc.robot.commands.OutakeCone;
import frc.robot.commands.OutakeCube;
import frc.robot.commands.OutakeCubeFast;
import frc.robot.commands.ScoreCubeHigh;
import frc.robot.commands.ScoreHigh;
import frc.robot.commands.ScoreLow;
import frc.robot.commands.ScoreMid;
import frc.robot.commands.ScoreMidCube;
import frc.robot.commands.ShootCube;
import frc.robot.commands.ShoulderToHome;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristSubsystem;

//walker was here
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RightScore3ByLoader extends SequentialCommandGroup {
  /** Creates a new Auto1. */
  public RightScore3ByLoader(Swerve s_Swerve, IntakeSubsystem s_Intake, ArmSubsystem s_Arm,
      WristSubsystem s_Wrist) {
    addCommands(
        
        (new ScoreMid(s_Wrist, s_Arm).alongWith(new IntakeCone(s_Intake))).withTimeout(0.8),
        (new OutakeCube(s_Intake)).withTimeout(1.5),
        (new ArmToHome(s_Wrist, s_Arm))
            .withTimeout(.5),
        new PathPlannerCommand(s_Swerve, 2, "Back up after cone top", true)
            .deadlineWith(new ArmToHome(s_Wrist, s_Arm).andThen(new ShoulderToHome(s_Arm))),
        new PathPlannerCommand(s_Swerve, 1, "Grab first cube top")
             .deadlineWith(new ScoreLow(s_Wrist, s_Arm),
                new IntakeCube(s_Intake),
                new InstantCommand(() -> s_Arm.shoulderReversed *= -1)),
        new PathPlannerCommand(s_Swerve, 3, "Score first cube top")
            .deadlineWith(new IntakeCube(s_Intake).withTimeout(1),
                new InstantCommand(() -> s_Arm.shoulderReversed *= -1),
                new ArmToHome(s_Wrist, s_Arm)
                    .andThen(new ShoulderToHome(s_Arm))),
        new PathPlannerCommand(s_Swerve, 2, "Line up top"),
        //new ScoreMidCube(s_Wrist, s_Arm).withTimeout(1.5),
        new ScoreMidCube(s_Wrist, s_Arm).withTimeout(1.5),
        new OutakeCube(s_Intake).withTimeout(1)
            .deadlineWith(new ScoreMid(s_Wrist, s_Arm)),
        new ArmToHome(s_Wrist, s_Arm).withTimeout(.5),
        new PathPlannerCommand(s_Swerve, 2, "Go back out top loader")

        
        /*new PathPlannerCommand(s_Swerve, 4, "Grab cube")
            .deadlineWith(new SequentialCommandGroup(
                new ShoulderToHome(s_Arm),
                new InstantCommand(() -> s_Arm.shoulderReversed *= -1))),
                new ScoreLow(s_Wrist, s_Arm).withTimeout(0.8),
        new PathPlannerCommand(s_Swerve, 4, "Tiny cube").alongWith(new ScoreLow(s_Wrist, s_Arm)).alongWith(new IntakeCube(s_Intake)).withTimeout(1.5),
        new ArmToHome(s_Wrist, s_Arm).withTimeout(.5)
        new PathPlannerCommand(s_Swerve, 4, "Score cube")
            .alongWith(new ShoulderToHome(s_Arm),
                new InstantCommand(() -> s_Arm.shoulderReversed *= -1),
                new IntakeCone(s_Intake).withTimeout(2.5)),
        new ShootCube(s_Wrist, s_Arm).withTimeout(.5),
        new OutakeCubeFast(s_Intake).withTimeout(.5),
        new ArmToHome(s_Wrist, s_Arm).withTimeout(.5),
        new ShoulderToHome(s_Arm).withTimeout(.5)*/);
  }
}