// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BottomScore1CubeAnd1Cone extends SequentialCommandGroup {
  /** Creates a new Auto1. */
  public BottomScore1CubeAnd1Cone(Swerve swerve) {
    addCommands(
      new PathPlannerCommand(swerve, 1, "Go to first game piece 1 bottom"),
      new PathPlannerCommand(swerve, 1, "Turn 180 Right 1 Bottom"),
      new PathPlannerCommand(swerve, 1, "Go to first game piece 2 bottom"),
      new PathPlannerCommand(swerve, 1, "Turn 180 left 1 Bottom"), 
      new PathPlannerCommand(swerve, 1, "Go back Bottom"),
      new PathPlannerCommand(swerve, 1, "Score cone 1 Bottom")
    );
  }
}
