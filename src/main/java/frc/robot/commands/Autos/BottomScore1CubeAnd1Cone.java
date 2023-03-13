package frc.robot.commands.Autos;

import java.time.Instant;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmToHome;
import frc.robot.commands.IntakeCone;
import frc.robot.commands.IntakeCube;
import frc.robot.commands.OutakeCube;
import frc.robot.commands.ScoreHigh;
import frc.robot.commands.ScoreLow;
import frc.robot.commands.ScoreMid;
import frc.robot.commands.ShoulderToHome;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristSubsystem;
//walker was here
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BottomScore1CubeAnd1Cone extends SequentialCommandGroup {
  /** Creates a new Auto1. */
  public BottomScore1CubeAnd1Cone(Swerve s_Swerve, IntakeSubsystem s_Intake, ArmSubsystem s_Arm, WristSubsystem s_Wrist) {
    addCommands(
      
    (new ScoreHigh(s_Arm, s_Wrist).alongWith(new IntakeCone(s_Intake))).withTimeout(2),
    (new OutakeCube(s_Intake)).withTimeout(.5),
    /*(new ArmToHome(s_Wrist, s_Arm))
        .withTimeout(.5),*/
    new PathPlannerCommand(s_Swerve, 4, "Back up after cone bottom", true)
/*      .alongWith(new InstantCommand(() -> s_Arm.shoulderReversed *= -1))
      .alongWith(new ScoreLow(s_Wrist, s_Arm))
      .withTimeout(2),*/
      .deadlineWith(new SequentialCommandGroup(
        new InstantCommand(() -> s_Arm.shoulderReversed *= -1),
        new ArmToHome(s_Wrist, s_Arm),
          new ShoulderToHome(s_Arm),
          new ScoreLow(s_Wrist, s_Arm))),
    new PathPlannerCommand(s_Swerve, 2, "Grab cube 1 Bottom")
      .alongWith(new IntakeCube(s_Intake))
        .alongWith(new ScoreLow(s_Wrist, s_Arm))
        .withTimeout(3),
    new PathPlannerCommand(s_Swerve, 4, "Go back Bottom 1")
      .alongWith (new ShoulderToHome(s_Arm))
        .withTimeout(2),
    new PathPlannerCommand(s_Swerve, 2, "Go back Bottom 2")
      .alongWith(new ScoreHigh(s_Arm, s_Wrist))
        .withTimeout(1.5)
        .alongWith(new InstantCommand(() -> s_Arm.shoulderReversed *= -1)),
    (new OutakeCube(s_Intake)).withTimeout(.5),
    new PathPlannerCommand(s_Swerve, 4, "Go back after cube Bottom").alongWith(new InstantCommand(() -> s_Arm.shoulderReversed *= -1)).alongWith(new ScoreLow(s_Wrist, s_Arm)).alongWith(new IntakeCube(s_Intake)).withTimeout(3.5),
    new PathPlannerCommand(s_Swerve, 4, "Score second cube Bottom").alongWith (new ShoulderToHome(s_Arm)).withTimeout(3),
    (new ScoreMid(s_Wrist, s_Arm)).withTimeout(1.5).alongWith(new InstantCommand(() -> s_Arm.shoulderReversed *= -1)),
    (new OutakeCube(s_Intake)).withTimeout(.5)
    );
  }
}