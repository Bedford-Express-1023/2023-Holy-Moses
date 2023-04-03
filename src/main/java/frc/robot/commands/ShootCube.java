// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ShootCube extends CommandBase {
  public WristSubsystem wrist;
  public ArmSubsystem arm;


  public ShootCube(WristSubsystem wrist, ArmSubsystem arm) {
    this.wrist = wrist;
    this.arm = arm;
    addRequirements(wrist, arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.shoulderPositionOverride = 0;
    arm.armPositionOverride = 0;
    wrist.wristPositionOverride = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*if (button.getAsBoolean() != booleanReverse) {
      booleanReverse = button.getAsBoolean();
      if (booleanReverse) {
        arm.shoulderReversed *= -1;
      }
    }*/

    wrist.wristPosition((arm.shoulderReversed * 55 - arm.shoulderCANCoder.getAbsolutePosition()));
    arm.ShoulderPosition(arm.shoulderReversed * arm.shoulderTargetAngleFeeder);
    arm.ArmPosition(arm.armTargetPositionFeeder);
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
