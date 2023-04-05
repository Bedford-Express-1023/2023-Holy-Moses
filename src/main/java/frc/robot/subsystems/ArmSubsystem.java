// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.RotationalFeedForward;
import frc.robot.Constants;
import static frc.robot.Constants.Arm.*;

public class ArmSubsystem extends SubsystemBase { 
  //shoulder is rotation (up/down)
  //arm is extension (in/out)
  public final TalonFX rearShoulderMotor = new TalonFX(Constants.Arm.FRONT_SHOULDER_CAN);
  public final TalonFX frontShoulderMotor = new TalonFX(Constants.Arm.REAR_SHOULDER_CAN);
  public final WPI_TalonFX armMotor = new WPI_TalonFX(Constants.Arm.ARM_EXTEND_CAN);
  //public final DigitalInput armLimitSwitch = new DigitalInput(Constants.Arm.ARM_LIMIT_SWITCH_DIO);

  public final CANCoder shoulderCANCoder = new CANCoder(Constants.Arm.SHOULDER_CANCODER);
  public final CANCoder armCANCoder = new CANCoder(Constants.Arm.ARM_CANCODER);

  public final PigeonIMU pidgeonGyro = new PigeonIMU(0);
  private final XboxController oliviaController = new XboxController(1);

  public double shoulderPosition = 0;
  public double shoulderPositionOverride = 0;
  public double armPosition = 0;
  public double armPositionOverride = 0;
  public int shoulderReversed = -1;

  public final double shoulderGravity = .07;
  public final double armGravity = -.4;
  public final RotationalFeedForward feedForward = new RotationalFeedForward(0,1,0, shoulderGravity);
  final PIDController armPID = new PIDController(0.006, 0.0, 0.00001);
  final PIDController shoulderPositionPID = new PIDController(.0195, 0.0, 0);

	final public double shoulderTargetAngleHigh = 50;
  final public double shoulderTargetAngleMiddle = 61;
  final public double shoulderTargetAngleLow = 120;
  public final double shoulderTargetAngleFeeder = 27;

  public final double armTargetPositionHigh = 1050;
  public final double armTargetPositionMiddle = 585.9;
  public final double armTargetPositionLow = 58.6;
  public final double armTargetPositionFeeder = 370;
  public final double armTargetPositionHome = 250;
  
  // Creates a new ArmSubsystem. 
  public ArmSubsystem() {
		rearShoulderMotor.setNeutralMode(NeutralMode.Brake);
		frontShoulderMotor.setNeutralMode(NeutralMode.Brake);
    frontShoulderMotor.follow(rearShoulderMotor);
    armPID.disableContinuousInput();
    armMotor.setNeutralMode(NeutralMode.Brake);
    armMotor.configOpenloopRamp(.2);
    armMotor.setInverted(false);
    armMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 50), 30);
    armMotor.configPeakOutputForward(1);
		armMotor.configPeakOutputReverse(-1);
  }

  /**
   * sets the shoulder position to the stored position
   * probably use the other overload though
   */
  private void ShoulderPosition() {
    //double vSetpoint = shoulderPositionPID.calculate(shoulderCANCoder.getAbsolutePosition(), shoulderPosition) * .5;
    //vSetpoint += shoulderVelocityPID.calculate(shoulderCANCoder.getVelocity(), vSetpoint);
    double output = shoulderPositionPID.calculate(shoulderCANCoder.getAbsolutePosition(), shoulderPosition);

    SmartDashboard.putNumber("gravity", shoulderGravity * -Math.sin(shoulderCANCoder.getAbsolutePosition()));
    SmartDashboard.putNumber("Shoulder output", output);
    rearShoulderMotor.set(ControlMode.PercentOutput, shoulderGravity * -Math.sin(shoulderCANCoder.getAbsolutePosition() * Math.PI/180) + 
      MathUtil.clamp(
        output,
        -Math.abs(-feedForward.calculate(MathUtil.clamp(output, -maxShoulderVelocity, maxShoulderVelocity), -maxShoulderAcceleration)),
        Math.abs(feedForward.calculate(MathUtil.clamp(output, -maxShoulderVelocity, maxShoulderVelocity), maxShoulderAcceleration)))); //calculates max power output so as not to go above max velocity and max accel //calculates max power output so as not to go above max velocity and max accel
  }

  public void ShoulderPosition(double positionShoulder) {
    shoulderPosition = positionShoulder;
  }

  private void ArmPosition() {
    double output = armPID.calculate(armCANCoder.getPosition(), armPosition + armPositionOverride);
    if (armPosition == armTargetPositionHome && Math.abs(armCANCoder.getPosition() - armPosition) < 175 && armPositionOverride == 0) {
      armMotor.set(0);
      return;
    }
    SmartDashboard.putNumber("ExtensionOutput", output);
    if (armCANCoder.getPosition() < 0 && output < 0) {
      armMotor.set(oliviaController.getLeftY() * .05);} else {
      armMotor.set(ControlMode.PercentOutput, armGravity * Math.cos(shoulderCANCoder.getAbsolutePosition() * Math.PI/180)
      -MathUtil.clamp(
        output,
        -Math.abs(feedForward.calculate(MathUtil.clamp(output, -maxArmVelocity, maxArmVelocity), -maxArmAcceleration)),
        Math.abs(feedForward.calculate(MathUtil.clamp(output, -maxArmVelocity, maxArmVelocity), maxArmAcceleration))));
    }
    //calculates max power output so as not to go above max velocity and max accel
  }

  public void ArmPosition(double positionArm) {
    armPosition = positionArm;
  }

  public void ArmPositionZero() {
    armMotor.setSelectedSensorPosition(0);
  }

  public void ArmToHome() {
    ArmPosition(armTargetPositionHome);
  }

  public void ShoulderToHome() {
    ShoulderPosition(0);
  }

  public boolean InPosition() {
    return Math.abs(shoulderCANCoder.getPosition() - shoulderPosition) < 5 && Math.abs(armCANCoder.getPosition() - armPosition) < 75;
  }

  public void ArmManual(double speed){
    armMotor.set(TalonFXControlMode.PercentOutput, speed);
  }

  /**
 * @param ticks the encoder value (in ticks, 2048/rotation)
 * @return the angle, in degrees, of the shoulder
 */
  public double TicksToDegrees(double ticks) {
    return (360 / 2048)/*ungeared ticks to degrees */ / (2048 / 18); /*gear ratio */
  }

  /**
  * @param ticks the encoder value in degrees
  * @return the distance, in inches, travelled by the arm
  */
  public double degreesToInches(double degrees) {
    return 2*Math.PI/*roller rotations to Inches of cord*/ * 2/*final pulley magic ratio*/ * (1/360)/*degrees to rotations*/ * (8/60);/*gear ratios*/
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShoulderPosition", shoulderCANCoder.getAbsolutePosition());
    SmartDashboard.putNumber("Arm Cancoder Pos", armCANCoder.getPosition());
    SmartDashboard.putNumber("ShoulderTarget", shoulderPosition);
    SmartDashboard.putNumber("ExtensionPosition", armMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("ExtensionTarget", armPosition);
    SmartDashboard.putNumber("ArmOverride", armPositionOverride);
    SmartDashboard.putNumber("ArmAmps", armMotor.getSupplyCurrent());
    SmartDashboard.putBoolean("Arm In Position?", InPosition());
    double leftYstick = oliviaController.getLeftY(); // left-side Y for Xbox360Gamepad 
		if (Math.abs(leftYstick) < 0.2) {
      leftYstick = 0; // deadband 15% 
    } 
    armPositionOverride -= leftYstick * 3;
    ArmPosition();
    //ShoulderPosition(0); //sets the arm to always be up
    ShoulderPosition();
  }
} 
