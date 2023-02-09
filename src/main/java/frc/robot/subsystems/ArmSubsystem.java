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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase { 
  //shoulder is rotation (up/down)
  //arm is extension (in/out)
  public final TalonFX leftShoulderMotor = new TalonFX(?);
  public final TalonFX rightShoulderMotor = new TalonFX(?);
  public final TalonFX armMotor = new TalonFX(?);

  public final CANCoder leftCANCoder = new CANCoder(?);
  public final CANCoder rightCANCoder = new CANCoder(?);
  public final CANCoder armCANCoder = new CANCoder(?);

  public final PigeonIMU pidgeonGyro = new PigeonIMU(0);

  public double shoulderPosition = 0;
  public final double maxShoulderVelocity = 15;
  public final double maxShoulderAcceleration = 15;
  public final double maxArmVelocity = 15;
  public final double maxArmAcceleration = 15;
  public final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(?, ?);
  final ProfiledPIDController armPID = new ProfiledPIDController(0.1, 0.0, 0.0, new Constraints(15, 100));
  final ProfiledPIDController shoulderPID = new ProfiledPIDController(0.1, 0.0, 0.0, new Constraints(15, 100));

	final double shoulderTargetAngleHigh = 0;
  final double shoulderTargetAngleMedium = 0;
  final double shoulderTargetAngleLow = 0;

  final double armTargetPositionHigh = 0;
  final double armTargetPositionMedium = 0;
  final double armTargetPositionLow = 0;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
		leftShoulderMotor.setNeutralMode(NeutralMode.Brake);
		rightShoulderMotor.setNeutralMode(NeutralMode.Brake);
    rightShoulderMotor.follow(leftShoulderMotor);

    ShuffleboardTab subsystemTab = Shuffleboard.getTab("Subsystems");
    ShuffleboardLayout rotationLayout = subsystemTab.getLayout("Arm Rotation")
    .withSize(1, 4)
    .withPosition(0, 2);
    rotationLayout.addDouble("RotationSpeed", () -> TicksToDegrees(leftCANCoder.getVelocity() * 10));
    rotationLayout.addDouble("RotationPosition", () -> TicksToDegrees(leftCANCoder.getAbsolutePosition()));
    
    ShuffleboardLayout extensionLayout = subsystemTab.getLayout("Arm Extension")
    .withSize(1, 4)
    .withPosition(0, 4);
    extensionLayout.addDouble("ExtensionSpeed", () -> armCANCoder.getVelocity() * 10);
    extensionLayout.addDouble("ExtensionPosition", () -> armCANCoder.getAbsolutePosition());
  }

  public void ShoulderPosition() {
    double vSetpoint = shoulderPID.calculate(2048*shoulderPosition/360) / 360;
    rightShoulderMotor.set(ControlMode.PercentOutput, 
      MathUtil.clamp(
        vSetpoint,
        -feedForward.calculate(MathUtil.clamp(vSetpoint, -maxShoulderVelocity, maxShoulderVelocity), maxShoulderAcceleration),
        feedForward.calculate(MathUtil.clamp(vSetpoint, -maxShoulderVelocity, maxShoulderVelocity), maxShoulderAcceleration))); //calculates max power output so as not to go above max velocity and max accel
    leftShoulderMotor.set(ControlMode.PercentOutput, 
      MathUtil.clamp(
        vSetpoint,
        -feedForward.calculate(MathUtil.clamp(vSetpoint, -maxShoulderVelocity, maxShoulderVelocity), maxShoulderAcceleration),
        feedForward.calculate(MathUtil.clamp(vSetpoint, -maxShoulderVelocity, maxShoulderVelocity), maxShoulderAcceleration))); //calculates max power output so as not to go above max velocity and max accel
  }

  public void ArmPosition() {
    double vSetpoint = armPID.calculate(2048*shoulderPosition/360) / 360;
    armMotor.set(ControlMode.PercentOutput, 
      MathUtil.clamp(
        vSetpoint,
        -feedForward.calculate(MathUtil.clamp(vSetpoint, -maxArmVelocity, maxArmVelocity), maxArmAcceleration),
        feedForward.calculate(MathUtil.clamp(vSetpoint, -maxArmVelocity, maxArmVelocity), maxArmAcceleration))); //calculates max power output so as not to go above max velocity and max accel
  }

  public void ShoulderPosition(double angle) {
    shoulderPosition = angle;
  }

  public void ArmHighScore() {
    shoulderPosition = shoulderTargetAngleHigh;
  }

  public void ArmMiddleScore() {
    shoulderPosition = shoulderTargetAngleMedium;
  }

  public void ArmLowScore() {
    shoulderPosition = shoulderTargetAngleLow;
  }
/**
 * @param ticks the encoder value (in ticks, 2048/rotation)
 * @return the angle, in degrees, off of absolute 0
 */
  public double TicksToDegrees(double ticks) {
    return (360 / 2048)/*ungeared ticks to degrees */ / (2048 / 18); /*gear ratio */
  }

  /**
 * @param ticks the encoder value (in ticks, 2048/rotation)
 * @return the distance, in meters, travelled by the arm
 */
  public double TicksToMeters(double ticks) {
    return (1/2048)/*ticks to rotations*/ * (8/60) * (18/35)/*gear ratios*/ * 0.0508*Math.PI/*roller rotations to meters of cord*/ * 2/*final pulley magic ratio*/;
  }

  @Override
  public void periodic() {
    ShoulderPosition(shoulderPosition);
  }
}