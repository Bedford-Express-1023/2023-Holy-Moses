// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Blinkin;

public class ShoulderLED extends CommandBase {
  /** Creates a new ShoulderDirection. */
  public ArmSubsystem arm;
  public Blinkin blinkin;
  public ShoulderLED(Blinkin blinkin, ArmSubsystem arm) {
    this.arm = arm;
   this.blinkin = blinkin;
   addRequirements(blinkin);
  }

   // public ArmSubsystem arm;
    //public Blinkin blinkinA;
    //public Blinkin blinkinB;
    //public ShoulderLED(Blinkin blinkinA, Blinkin blinkinB, ArmSubsystem arm) {
    //this.arm = arm;
    //this.blinkinA = blinkinA;
    //this.blinkinB = blinkinB;
    //addRequirements(blinkinA, blinkinB);}
    // Use addRequirements() here to declare subsystem dependencies.

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (arm.shoulderReversed == 1){
      blinkin.color();
    //  blinkinB.off();
    }
    else if (arm.shoulderReversed == -1){
      blinkin.off();
     // blinkinB.color();
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
