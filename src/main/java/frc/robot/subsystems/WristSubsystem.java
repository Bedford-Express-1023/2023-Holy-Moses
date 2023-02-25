// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;

import static com.revrobotics.CANSparkMax.SoftLimitDirection.*;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Intake.*;
import static frc.robot.Constants.Arm.*;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
  private CANSparkMax wristMotor = new CANSparkMax(WRIST_SPARK, kBrushless);
  private CANCoder wristCancoder = new CANCoder(WRIST_CANCODER);
  public RelativeEncoder wristEncoder;

  public double wristPosition = 0;
  public double wristPositionOverride = 0;

  private SparkMaxPIDController wristPidController;
  private SimpleMotorFeedforward wristFeedforward = new SimpleMotorFeedforward(0.0, 1, 0); //TODO: Tune
  private PIDController wristPID = new PIDController(0.005, 0.0, 0.0);

  public WristSubsystem() {
    wristEncoder = wristMotor.getEncoder();
    wristEncoder.setPositionConversionFactor(((72/32)/25) * 360);
    wristEncoder.setPosition(wristCancoder.getAbsolutePosition());
    wristPID.disableContinuousInput();
    wristMotor.setSmartCurrentLimit(30); //sets current limit to 20 amps
    wristMotor.setControlFramePeriodMs(20);
  }

  public void wristPosition() {
    double setpoint = -wristPID.calculate(wristCancoder.getAbsolutePosition(), wristPosition + wristPositionOverride);
    SmartDashboard.putNumber("WristPID setpoint", setpoint);
    if (wristCancoder.getAbsolutePosition() > 90 && setpoint > 0) {wristMotor.set(0);}
    else if (wristCancoder.getAbsolutePosition() < 90 && setpoint < 0) {wristMotor.set(0);}
    wristMotor.set(setpoint);
  }

  public void wristPosition(double position){
    wristPosition = position;
  }

  @Override
  public void periodic() {
    wristEncoder.setPosition(wristCancoder.getAbsolutePosition());
    wristPosition();
    SmartDashboard.putNumber("Wrist Output", wristMotor.getOutputCurrent());
    SmartDashboard.putNumber("Wrist Motor Position", wristMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Wrist Cancoder", wristCancoder.getAbsolutePosition());
    SmartDashboard.putNumber("wristTarget", wristPosition);
  }
}