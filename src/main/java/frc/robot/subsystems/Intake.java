// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {

  private final Solenoid solenoid = new Solenoid(null, 0);// find out channel
  private final VictorSPX intakeMotor = new VictorSPX(0);// find out device number

  /** Creates a new Intake. */
  public Intake() {}

public void intakeIn () {
  intakeMotor.set(VictorSPXControlMode.PercentOutput, -0.5);
}

public void intakeOut () {
  intakeMotor.set(VictorSPXControlMode.PercentOutput, 0.5);
}

public void intakeOff () {
  intakeMotor.set(VictorSPXControlMode.PercentOutput, 0);
}





  @Override
  public void periodic() {
    
    // This method will be called once per scheduler run
  }
}
