// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

public class AlignToTarget extends CommandBase {

  public PIDController LimeLightRotationPID;
  private Swerve s_Swerve;
  private Limelight s_Limelight;

  /** Creates a new AlignToTarget. */
  public AlignToTarget(Swerve swerve, Limelight limelight) {
    this.s_Swerve = swerve;
    this.s_Limelight = limelight;
    addRequirements(s_Swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    LimeLightRotationPID = new PIDController(6.5, 0.0, 0.005);
    LimeLightRotationPID.enableContinuousInput(-Math.PI, Math.PI);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (s_Limelight.updateLimelightVLeft(3) == 1) {
      double toRotate = LimeLightRotationPID.calculate(0, -s_Limelight.updateLimelightXLeft(3) * Math.PI/180);
      s_Swerve.drive(new Translation2d(0, 0), toRotate, false, false);
  }
    else if (s_Limelight.updateLimelightVLeft(3) == 0){
      return;
    }

  if (s_Limelight.limelightXLeft > -1 && s_Limelight.limelightXLeft < 1 ) {
      s_Swerve.drive(new Translation2d(0.0, 0.0), 0.0, false, false);  
  }
  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.drive(new Translation2d(0.0, 0.0), 0.0, false, false);  

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
