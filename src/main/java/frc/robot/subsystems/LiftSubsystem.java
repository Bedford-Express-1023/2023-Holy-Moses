// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LiftSubsystem extends SubsystemBase {
  private final CANSparkMax liftMotor = new CANSparkMax(0, MotorType.kBrushless);

  /** Creates a new LiftSubsystem. */
  public LiftSubsystem() {
  }

  public void liftUp() {
    liftMotor.set(0);
  }

  public void liftDown() {
    liftMotor.set(0);
  }

  public void liftScoreHigh() {
    liftMotor.set(0);
  }

  public void liftScoreMedium() {
    liftMotor.set(0);
  }

  public void liftScoreLow() {
    liftMotor.set(0);
  }

  public void liftStop() {
    liftMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
