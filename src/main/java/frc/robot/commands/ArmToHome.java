// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.WristSubsystem;

public class ArmToHome extends CommandBase {
  public WristSubsystem wrist;
  public ArmSubsystem arm;
  public Trigger button;
  public Boolean booleanReverse = true;

  public ArmToHome(WristSubsystem wrist, ArmSubsystem arm) {
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
    arm.ArmToHome();
    wrist.wristToHome();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    new ShoulderToHome(arm).schedule();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(arm.armTargetPositionHome - arm.armMotor.getSelectedSensorPosition()) < 5000;
  }
}
