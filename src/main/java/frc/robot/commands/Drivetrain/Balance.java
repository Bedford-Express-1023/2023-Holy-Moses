// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class Balance extends CommandBase {
  private Swerve s_Swerve;

  /** Creates a new Balance. */
  public Balance(Swerve s_Swerve) {
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (s_Swerve.gyro.getRoll() >= 7) {
      s_Swerve.drive(new Translation2d(-0.7, 0),
      0,
      false,
      true);
      SmartDashboard.putString("Balancing?", "balancing");
    }
    else if (s_Swerve.gyro.getRoll() <= -7) {
      s_Swerve.drive(new Translation2d(0.7, 0),
      0,
      false,
      true);
      SmartDashboard.putString("Balancing?", "balancing");
    }
    else {
      s_Swerve.drive(new Translation2d(0, 0),
      0,
      false,
      true);
      SmartDashboard.putString("Balancing?", "not balancing");

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
