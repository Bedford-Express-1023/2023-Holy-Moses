// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.SoftLimitDirection;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Intake;

import static frc.robot.Constants.Arm.*;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
  private CANSparkMax wristMotor = new CANSparkMax(WRIST_SPARK, kBrushless);
  private CANCoder wristCancoder = new CANCoder(WRIST_CANCODER);
  public RelativeEncoder wristEncoder;
  private final XboxController oliviaController = new XboxController(1);


  public double wristPosition = 0;
  public double wristPositionOverride = 0;

  private SimpleMotorFeedforward wristFeedforward = new SimpleMotorFeedforward(0.0, 1, 0); //TODO: Tune
  private PIDController wristPID = new PIDController(0.010, 0.0, 0.0);
  public double wristGravity = -0.1;

  public WristSubsystem() {
    wristEncoder = wristMotor.getEncoder();
    wristEncoder.setPositionConversionFactor(((72/32)/25) * 360);
    wristEncoder.setPosition(wristCancoder.getAbsolutePosition());
    wristPID.disableContinuousInput();
    wristMotor.setSoftLimit(SoftLimitDirection.kReverse, -110);
    wristMotor.setSoftLimit(SoftLimitDirection.kForward, 110);
    wristMotor.setSmartCurrentLimit(30); //sets current limit to 20 amps
    wristMotor.setControlFramePeriodMs(20);
  }

  public void wristPosition() {
    double output = wristPID.calculate(wristCancoder.getAbsolutePosition(), wristPosition + Math.signum(wristPosition) * wristPositionOverride);
    SmartDashboard.putNumber("WristPID setpoint", output);
    if (wristCancoder.getAbsolutePosition() > 100 && output < 0) {wristMotor.set(0); return;}
    else if (wristCancoder.getAbsolutePosition() < -100 && output > 0) {wristMotor.set(0); return;}
    wristMotor.set((wristGravity * -Math.sin(Math.toRadians(-wristPosition + 80 * Math.signum(wristPosition) + wristCancoder.getAbsolutePosition()))) + //gravity
    MathUtil.clamp(output, -Intake.maxWristVelocity, Intake.maxWristVelocity ));
  }

  public void wristPosition(double position) {
    wristPosition = position;
  }

  public void wristToHome() {
    wristPosition(0);
  }

  @Override
  public void periodic() {
    if (Math.abs(oliviaController.getRightY()) > .2) {
      wristPositionOverride += oliviaController.getRightY() * 0.5;
    }

    wristEncoder.setPosition(wristCancoder.getAbsolutePosition());
    //wristPosition();
    SmartDashboard.putNumber("Wrist Override", wristPositionOverride);
    SmartDashboard.putNumber("Wrist Output", wristMotor.getOutputCurrent());
    SmartDashboard.putNumber("Wrist Motor Position", wristMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Wrist Cancoder", wristCancoder.getAbsolutePosition());
    SmartDashboard.putNumber("wristTarget", wristPosition);
  }
}