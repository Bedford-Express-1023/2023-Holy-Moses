// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmToHome;
import frc.robot.commands.IntakeCube;
import frc.robot.commands.OutakeCone;
import frc.robot.commands.OutakeCube;
import frc.robot.commands.ScoreHigh;
import frc.robot.commands.ScoreLow;
import frc.robot.commands.ShoulderToHome;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.WristSubsystem;
//walker was here
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ChargingStation extends SequentialCommandGroup {
  /** Creates a new Auto1. */
  public ChargingStation(Swerve swerve) {
    addCommands(
      new PathPlannerCommand(swerve, 5, "Charging station")
    );
  }
}

