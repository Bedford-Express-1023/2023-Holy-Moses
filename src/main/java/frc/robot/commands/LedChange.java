// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Blinkin;

public class LedChange extends CommandBase {
  Blinkin blinkin;
  public ArmSubsystem arm;
  /** Creates a new LedChange. */
  public LedChange(Blinkin blinkin, ArmSubsystem arm) {
    this.blinkin = blinkin;
    this.arm = arm;
    addRequirements(blinkin);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void changeColor (){
  if (arm.shoulderReversed == -1 && DriverStation.getAlliance() == Alliance.Blue){
     blinkin.blue1();
     blinkin.off2();
    }
  else if(arm.shoulderReversed == 1 && DriverStation.getAlliance() == Alliance.Blue){
    blinkin.blue2();
    blinkin.off1();
  }
  else if(arm.shoulderReversed == -1  && DriverStation.getAlliance() == Alliance.Red){
    blinkin.red1();
    blinkin.off2();
  }
  else if(arm.shoulderReversed == 1  && DriverStation.getAlliance() == Alliance.Red){
    blinkin.red2();
    blinkin.off1();
  }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   

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
