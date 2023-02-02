// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  public final TalonFX leftArmMotor = new TalonFX(?);
  public final TalonFX rightArmMotor = new TalonFX(?);
  public final CANCoder leftCANCoder = new CANCoder(?);
  public final CANCoder rightCANCoder = new CANCoder(?);

  public final PigeonIMU pidgeonGyro = new PigeonIMU(0);
  public final double maxVelocity = 15;
  public final double maxAcceleration = 15;
  public final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(?, ?);
  final ProfiledPIDController extensionPID = new ProfiledPIDController(0.1, 0.0, 0.0, new Constraints(15, 100));
  final ProfiledPIDController rotationPID = new ProfiledPIDController(0.1, 0.0, 0.0, new Constraints(15, 100));

	final double targetAngleHigh = 0;
  final double targetAngleMedium = 0;
  final double targetAngleLow = 0;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
		leftArmMotor.setNeutralMode(NeutralMode.Brake);
		rightArmMotor.setNeutralMode(NeutralMode.Brake);
    rightArmMotor.follow(leftArmMotor);

    ShuffleboardTab armTab = Shuffleboard.getTab("Arm");
    ShuffleboardLayout rotationLayout = armTab.getLayout("Rotation");
    rotationLayout.addDouble("RotationSpeed", () -> leftCANCoder.getVelocity()*360*10/2048);
    rotationLayout.addDouble("RotationPosition", () -> leftCANCoder.getAbsolutePosition()*360/2048);
    
    ShuffleboardLayout extensionLayout = armTab.getLayout("Extension");
    rotationLayout.addDouble("ExtensionSpeed", () -> leftCANCoder.getVelocity()*10/2048);
    rotationLayout.addDouble("ExtensionPosition", () -> leftCANCoder.getAbsolutePosition()/2048);

  }

  public void armPosition(double angle) {
    double vSetpoint = rotationPID.calculate(2048*angle/360) / 360;
    rightArmMotor.set(ControlMode.PercentOutput, 
      MathUtil.clamp(
        rotationPID.calculate(2048*angle/360) / 360,
        -feedForward.calculate(MathUtil.clamp(vSetpoint, -maxVelocity, maxVelocity), maxAcceleration),
        feedForward.calculate(MathUtil.clamp(vSetpoint, -maxVelocity, maxVelocity), maxAcceleration))); //calculates max power output so as not to go above max velocity and max accel

    leftArmMotor.set(ControlMode.PercentOutput, 
      MathUtil.clamp(
        rotationPID.calculate(2048*angle/360) / 360,
        -feedForward.calculate(MathUtil.clamp(vSetpoint, -maxVelocity, maxVelocity), maxAcceleration),
        feedForward.calculate(MathUtil.clamp(vSetpoint, -maxVelocity, maxVelocity), maxAcceleration))); //calculates max power output so as not to go above max velocity and max accel
  }

  public void armHighAngle() {
  }

  public void armMiddleAngle() {
  }

  public void armLowAngle() {
  }

  @Override
  public void periodic() {
    armPosition(askdaksdjasjdkasjd);
  }
}
