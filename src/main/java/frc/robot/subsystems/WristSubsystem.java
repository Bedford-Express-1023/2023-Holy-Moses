// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import static com.revrobotics.CANSparkMax.SoftLimitDirection.*;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Intake.*;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
  private CANSparkMax wristSpark = new CANSparkMax(WRIST_SPARK, kBrushless);
  private CANCoder wristCancoder = new CANCoder(WRIST_CANCODER);
  private RelativeEncoder neoEncoder = wristSpark.getEncoder();

  private ArmFeedforward wristFeedforward = new ArmFeedforward(0.1, 0.1, 0.1);
  private PIDController wristPID = new PIDController(0.1, 0.0, 0.0);

  public WristSubsystem() {
    wristSpark.setSoftLimit(kForward, 240); //sets forward limit to 238 on internal encoder
    wristSpark.setSoftLimit(kReverse, 0); //sets reverse limit to 0 on internal encoder
    wristSpark.setSmartCurrentLimit(20); //sets current limit to 20 amps
    neoEncoder.setPosition(0.0);
  }

  public void setWrist(double position){
    
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Cancoder position", wristCancoder.getAbsolutePosition());
    SmartDashboard.putNumber("Neo position", neoEncoder.getPosition());
    // This method will be called once per scheduler run
  }
}
