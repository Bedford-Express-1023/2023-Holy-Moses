// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import static com.revrobotics.CANSparkMax.SoftLimitDirection.*;

import static com.revrobotics.CANSparkMaxLowLevel.MotorType.*;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Intake.*;

public class WristSubsystem extends SubsystemBase {
  /** Creates a new WristSubsystem. */
  private CANSparkMax wristSpark = new CANSparkMax(wrist_SPARK_CAN, kBrushless);
  private CANCoder wristCancoder = new CANCoder(wrist_CANCODER);

  private ArmFeedforward wristFeedforward = new ArmFeedforward(0.1, 0.1, 0.1);
  private PIDController wristPID = new PIDController(0.1, 0.0, 0.0);

  public WristSubsystem() {
    wristSpark.setSoftLimit(kForward, 238); //sets forward limit to 238 on internal encoder
    wristSpark.setSoftLimit(kReverse, 0); //sets reverse limit to 0 on internal encoder
    wristSpark.setSmartCurrentLimit(20); //sets current limit to 20 amps
  }

  public void setWrist(double position){
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
