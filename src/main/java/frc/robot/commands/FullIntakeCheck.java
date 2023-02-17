// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.IntakeSubsystem;

public class FullIntakeCheck extends CommandBase {
  IntakeSubsystem m_IntakeSubsystem;
  Blinkin m_Blinkin;
  /** Creates a new FullIntakeCheck. */
  public FullIntakeCheck(IntakeSubsystem intakeSubsystem, Blinkin blinkin) {
    m_IntakeSubsystem = intakeSubsystem;
    m_Blinkin = blinkin;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_IntakeSubsystem, m_Blinkin);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_IntakeSubsystem.FullIntakeCheck();
    if(m_IntakeSubsystem.FullIntakeCheck = true) {
      
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
