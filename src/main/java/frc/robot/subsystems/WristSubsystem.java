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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Intake.*;
import static frc.robot.Constants.Arm.*;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
  private CANSparkMax wristSpark = new CANSparkMax(WRIST_SPARK, kBrushless);
  private CANCoder wristCancoder = new CANCoder(WRIST_CANCODER);
  private RelativeEncoder neoEncoder = wristSpark.getEncoder();

  private SparkMaxPIDController wristPidController;

  private ArmFeedforward wristFeedforward = new ArmFeedforward(0.1, 0.1, 0.1);
  private PIDController wristPID = new PIDController(0.1, 0.0, 0.0);

  public WristSubsystem() {
    wristSpark.setSoftLimit(kForward, 0); 
    wristSpark.setSoftLimit(kReverse, -265);
    wristSpark.setSmartCurrentLimit(30); //sets current limit to 20 amps
    neoEncoder.setPosition(0.0);
    wristSpark.setControlFramePeriodMs(20);

    wristPidController = wristSpark.getPIDController();
    wristPidController.setP(2);
    wristPidController.setI(0.0);
    wristPidController.setD(0.0);
    wristPidController.setIZone(0.0);
    wristPidController.setFF(.5);
    wristPidController.setOutputRange(-.6, .6);



  }

  public void setWrist(double position){
      wristPidController.setReference(position, CANSparkMax.ControlType.kPosition);
      /*if (neoEncoder.getPosition() >= position + 2 && neoEncoder.getPosition() <= position -2){
        stopWrist();
      }
      else {
        return;
      }*/
  
  }


  public void stopWrist(){
    wristSpark.stopMotor();
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("Cancoder position", wristCancoder.getAbsolutePosition());
    SmartDashboard.putNumber("Neo position", neoEncoder.getPosition());
    SmartDashboard.putNumber("Wrist output", wristSpark.getOutputCurrent());
    // This method will be called once per scheduler run
  }
}