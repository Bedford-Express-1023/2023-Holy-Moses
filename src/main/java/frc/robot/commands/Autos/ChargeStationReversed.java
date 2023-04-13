// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.ArmToHome;
import frc.robot.commands.IntakeCone;
import frc.robot.commands.IntakeCube;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.OutakeCone;
import frc.robot.commands.OutakeCube;
import frc.robot.commands.ScoreHigh;
import frc.robot.commands.ScoreLow;
import frc.robot.commands.ShoulderToHome;
import frc.robot.commands.Drivetrain.Balance;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristSubsystem;
//walker was here
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChargeStationReversed extends SequentialCommandGroup {
  /** Creates a new Auto1. */
  public ChargeStationReversed(Swerve s_Swerve, IntakeSubsystem s_Intake, ArmSubsystem s_Arm, WristSubsystem s_Wrist) {
    addCommands(
      
    (new ScoreHigh(s_Arm, s_Wrist).alongWith(new IntakeCone(s_Intake))).withTimeout(1.5),
    (new OutakeCube(s_Intake)).withTimeout(.5),
   (new ArmToHome(s_Wrist, s_Arm)).withTimeout(1),
  (new ShoulderToHome(s_Arm)).withTimeout(1),
    new PathPlannerCommand(s_Swerve, 2, "Over charge station", true),
    new WaitCommand(.5),
    new PathPlannerCommand(s_Swerve, 2, "Charging station"),
    (new Balance(s_Swerve))
    //.alongWith(new InstantCommand(() -> s_Arm.shoulderReversed *= -1)).alongWith(new ScoreLow(s_Wrist, s_Arm)).withTimeout(1)
    //new PathPlannerCommand(s_Swerve, 2, "Grab cube charge station").alongWith(new IntakeCube(s_Intake)).withTimeout(3),
    //new PathPlannerCommand(s_Swerve, 4, "Score cube charge station fast").alongWith(new ShoulderToHome(s_Arm)).withTimeout(2),
    //new PathPlannerCommand(s_Swerve, 4, "Score cube charge station slow").alongWith(new ScoreHigh(s_Arm, s_Wrist)).withTimeout(1.5).alongWith(new InstantCommand(() -> s_Arm.shoulderReversed *= -1)),
    //(new OutakeCube(s_Intake)).withTimeout(.5),
    //new PathPlannerCommand(s_Swerve, 4, "Charging station"),
    //new PathPlannerCommand(s_Swerve, 4, "Little right")
    );
  }
}
