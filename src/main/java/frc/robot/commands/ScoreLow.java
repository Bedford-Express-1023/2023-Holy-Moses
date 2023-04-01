// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.PS4Controller.Button;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ScoreLow extends CommandBase {
  public WristSubsystem wrist;
  public ArmSubsystem arm;
  public Trigger button;
  public Boolean booleanReverse = true;


  public ScoreLow(WristSubsystem wrist, ArmSubsystem arm, Trigger button) {
    this.wrist = wrist;
    this.arm = arm;
    this.button = button;
    addRequirements(wrist, arm);
  }

  public ScoreLow(WristSubsystem wrist, ArmSubsystem arm) {
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
    arm.ShoulderPosition(arm.shoulderReversed * arm.shoulderTargetAngleLow);
    arm.ArmPosition(arm.armTargetPositionLow);
    wrist.wristPosition((arm.shoulderReversed * 90 - arm.shoulderCANCoder.getAbsolutePosition()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
