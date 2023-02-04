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
  public double armPosition = 0;
  public final double maxRotationVelocity = 15;
  public final double maxRotationAcceleration = 15;
  public final double maxExtensionVelocity = 15;
  public final double maxExtensionAcceleration = 15;
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
    rotationLayout.addDouble("RotationSpeed", () -> ticksToDegrees(leftCANCoder.getVelocity() * 10));
    rotationLayout.addDouble("RotationPosition", () -> ticksToDegrees(leftCANCoder.getAbsolutePosition()));
    
    ShuffleboardLayout extensionLayout = armTab.getLayout("Extension");
    rotationLayout.addDouble("ExtensionSpeed", () -> leftCANCoder.getVelocity()*10);
    rotationLayout.addDouble("ExtensionPosition", () -> leftCANCoder.getAbsolutePosition());

  }

  public void armRotationPosition() {
    double vSetpoint = rotationPID.calculate(2048*armPosition/360) / 360;
    rightArmMotor.set(ControlMode.PercentOutput, 
      MathUtil.clamp(
        vSetpoint,
        -feedForward.calculate(MathUtil.clamp(vSetpoint, -maxRotationVelocity, maxRotationVelocity), maxRotationAcceleration),
        feedForward.calculate(MathUtil.clamp(vSetpoint, -maxRotationVelocity, maxRotationVelocity), maxRotationAcceleration))); //calculates max power output so as not to go above max velocity and max accel
    leftArmMotor.set(ControlMode.PercentOutput, 
      MathUtil.clamp(
        vSetpoint,
        -feedForward.calculate(MathUtil.clamp(vSetpoint, -maxRotationVelocity, maxRotationVelocity), maxRotationAcceleration),
        feedForward.calculate(MathUtil.clamp(vSetpoint, -maxRotationVelocity, maxRotationVelocity), maxRotationAcceleration))); //calculates max power output so as not to go above max velocity and max accel
  }

  public void armExtensionPosition() {
    double vSetpoint = extensionPID.calculate(2048*armPosition/360) / 360;
    rightArmMotor.set(ControlMode.PercentOutput, 
      MathUtil.clamp(
        vSetpoint,
        -feedForward.calculate(MathUtil.clamp(vSetpoint, -maxExtensionVelocity, maxRotationVelocity), maxRotationAcceleration),
        feedForward.calculate(MathUtil.clamp(vSetpoint, -maxRotationVelocity, maxRotationVelocity), maxRotationAcceleration))); //calculates max power output so as not to go above max velocity and max accel
    leftArmMotor.set(ControlMode.PercentOutput, 
      MathUtil.clamp(
        vSetpoint,
        -feedForward.calculate(MathUtil.clamp(vSetpoint, -maxExtensionVelocity, maxExtensionVelocity), maxExtensionAcceleration),
        feedForward.calculate(MathUtil.clamp(vSetpoint, -maxExtensionVelocity, maxExtensionVelocity), maxExtensionAcceleration))); //calculates max power output so as not to go above max velocity and max accel
  }

  public void armPosition(double angle) {
    armPosition = angle;
  }

  public void armHighScore() {
    armPosition = targetAngleHigh;
  }

  public void armMiddleScore() {
    armPosition = targetAngleMedium;
  }

  public void armLowScore() {
    armPosition = targetAngleLow;
  }
/**
 * @param ticks the encoder value (in ticks, 2048/rotation)
 * @return the angle, in degrees, off of absolute 0
 */
  public double ticksToDegrees(double ticks) {
    return (360 / 2048)/*ungeared ticks to degrees */ / (2048 / 18); /*gear ratio */
  }

  /**
 * @param ticks the encoder value (in ticks, 2048/rotation)
 * @return the distance, in meters, travelled by the arm
 */
  public double ticksToMeters(double ticks) {
    return (1/2048)/*ticks to rotations*/ * (8/60) * (18/35)/*gear ratios*/ * 0.0508*Math.PI/*roller rotations to meters of cord*/ * 2/*final pulley magic ratio*/;
  }


  @Override
  public void periodic() {
    armPosition(armPosition);
  }
}
