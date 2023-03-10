// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCone;
import frc.robot.commands.OutakeCube;
import frc.robot.commands.ScoreHigh;
import frc.robot.commands.ScoreLow;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristSubsystem;
//walker was here
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TopScore1CubeAnd1Cone extends SequentialCommandGroup {
  /** Creates a new Auto1. */
  public TopScore1CubeAnd1Cone(Swerve s_Swerve, IntakeSubsystem s_Intake, ArmSubsystem s_Arm, WristSubsystem s_Wrist) {
    addCommands(
      
    (new ScoreHigh(s_Arm, s_Wrist).alongWith(new IntakeCone(s_Intake))).withTimeout(3),
    (new OutakeCube(s_Intake)).withTimeout(2),
    //(new OutakeCube(intakeSubsystem).withTimeout(2)),
    //(new ArmToHome(s_Wrist, s_Arm)).withTimeout(2),
    //(new ShoulderToHome(s_Arm)).withTimeout(3),
    new PathPlannerCommand(s_Swerve, 1, "Straight 1 Bottom"),
    new PathPlannerCommand(s_Swerve, 3, "Score 1 and go back fast", true),
    new PathPlannerCommand(s_Swerve, 1, "Score 1 and go back slow").alongWith(new InstantCommand(() -> s_Arm.shoulderReversed *= -1)).alongWith(new ScoreLow(s_Wrist, s_Arm)).withTimeout(3)
    );
  }
}

