// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static frc.robot.Constants.Arm.*;

public class ArmSubsystem extends SubsystemBase { 
  //shoulder is rotation (up/down)
  //arm is extension (in/out)
  public final TalonFX rearShoulderMotor = new TalonFX(Constants.Arm.REAR_SHOULDER_CAN);
  public final TalonFX frontShoulderMotor = new TalonFX(Constants.Arm.FRONT_SHOULDER_CAN);
  public final WPI_TalonFX armMotor = new WPI_TalonFX(Constants.Arm.ARM_EXTEND_CAN);

  public final CANCoder shoulderCANCoder = new CANCoder(Constants.Arm.SHOULDER_CANCODER);

  public final PigeonIMU pidgeonGyro = new PigeonIMU(0);
  private final XboxController oliviaController = new XboxController(1);

  public double shoulderPosition = 0;
  public double shoulderPositionOverride = 0;
  public double armPosition = 0;
  public double armPositionOverride = 0;
  public int shoulderReversed = -1;

  public final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0,1,0);
  final PIDController armPID = new PIDController(1.6, 0.0, 0.0);
  final PIDController shoulderPID = new PIDController(1.5, 0.0, 0.1);

	final public double shoulderTargetAngleHigh = 30;
  final public double shoulderTargetAngleMiddle = 30;
  final public double shoulderTargetAngleLow = 60;

  public final double armTargetPositionHigh = 0;
  public final double armTargetPositionMiddle = 0;
  public final double armTargetPositionLow = 0;

  public String currentShoulderCommand = "";  
  public String currentArmCommand = "";
  public final DigitalInput limitSwitch = new DigitalInput(1);
  public final Counter counter = new Counter(limitSwitch);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
		rearShoulderMotor.setNeutralMode(NeutralMode.Brake);
		frontShoulderMotor.setNeutralMode(NeutralMode.Brake);
    frontShoulderMotor.follow(rearShoulderMotor);
    armMotor.setNeutralMode(NeutralMode.Coast);


		rearShoulderMotor.configPeakOutputForward(+.5);
		rearShoulderMotor.configPeakOutputReverse(-.5);
		frontShoulderMotor.configPeakOutputForward(+.5);
		frontShoulderMotor.configPeakOutputReverse(-.5);
  
    armMotor.configPeakOutputForward(+.5);
		armMotor.configPeakOutputReverse(-.5);

    ShuffleboardTab subsystemTab = Shuffleboard.getTab("Subsystems");
    ShuffleboardLayout extensionLayout = subsystemTab.getLayout("Arm Extension", BuiltInLayouts.kList)
    .withSize(1, 4)
    .withPosition(0, 4);
    extensionLayout.addNumber("ExtensionSpeed", () -> armMotor.getSelectedSensorVelocity());
    extensionLayout.addNumber("ExtensionPosition", () -> armMotor.getSelectedSensorPosition());

    ShuffleboardLayout rotationLayout = subsystemTab.getLayout("Shoulder Rotation", BuiltInLayouts.kList)
    .withSize(1, 4)
    .withPosition(0, 2);
    rotationLayout.addDouble("RotationSpeed", () -> shoulderCANCoder.getVelocity());
    rotationLayout.addDouble("RotationPosition", () -> shoulderCANCoder.getPosition());
    rotationLayout.add("Current Arm Command", currentArmCommand, "none");
  }

  /**
   * sets the shoulder position to the stored position
   * probably use the other overload though
   */
  public void ShoulderPosition() {
    double vSetpoint = shoulderPID.calculate(shoulderCANCoder.getAbsolutePosition(), shoulderPosition);
    SmartDashboard.putNumber("PID", vSetpoint);
    frontShoulderMotor.set(ControlMode.PercentOutput, 
      MathUtil.clamp(
        vSetpoint,
        -Math.abs(feedForward.calculate(MathUtil.clamp(vSetpoint, -maxShoulderVelocity, maxShoulderVelocity), -maxShoulderAcceleration)),
        Math.abs(feedForward.calculate(MathUtil.clamp(vSetpoint, -maxShoulderVelocity, maxShoulderVelocity), maxShoulderAcceleration))/180)); //calculates max power output so as not to go above max velocity and max accel
    rearShoulderMotor.set(ControlMode.PercentOutput, 
      MathUtil.clamp(
        vSetpoint,
        -Math.abs(-feedForward.calculate(MathUtil.clamp(Math.abs(vSetpoint), -maxShoulderVelocity, maxShoulderVelocity), -maxShoulderAcceleration)),
        Math.abs(feedForward.calculate(MathUtil.clamp(Math.abs(vSetpoint), -maxShoulderVelocity, maxShoulderVelocity), maxShoulderAcceleration))/180)); //calculates max power output so as not to go above max velocity and max accel
  }

  /**
   * sets the arm position to the stored position
   * probably use the other overload though
   */
  public void ArmPosition() {
    double vSetpoint = armPID.calculate(TicksToMeters(armMotor.getSelectedSensorPosition()), armPosition);
    armMotor.set(ControlMode.PercentOutput,
     MathUtil.clamp(
      vSetpoint,
      -Math.abs(-feedForward.calculate(MathUtil.clamp(Math.abs(vSetpoint), -maxArmVelocity, maxArmVelocity), maxArmAcceleration)),
      Math.abs(feedForward.calculate(MathUtil.clamp(Math.abs(vSetpoint), -maxArmVelocity, maxArmVelocity), maxArmAcceleration))/180)); //calculates max power output so as not to go above max velocity and max accel
  }


  public void ShoulderPosition(double angle) {
    shoulderPosition = angle;
  }

  public void ArmPosition(double angle) {
    armPosition = angle;
  }

  public void ArmPercent(double percent) {
    armMotor.set(ControlMode.PercentOutput, percent);
  }

  public void ArmStop() {
    armMotor.set(ControlMode.PercentOutput, 0);
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

  public void setAngleSpeed(double speed) {
    if (counter.get() > 0) {
      rearShoulderMotor.set(TalonFXControlMode.PercentOutput, 0);
    } else {
      rearShoulderMotor.set(TalonFXControlMode.PercentOutput, speed);
    }
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("ShoulderPosition", shoulderCANCoder.getAbsolutePosition());
    SmartDashboard.putNumber("ExtenstionPosition", shoulderCANCoder.getAbsolutePosition()/1000);
    double leftYstick = -oliviaController.getRawAxis(0); // left-side Y for Xbox360Gamepad 
		double rghtYstick = -oliviaController.getRawAxis(1); // right-side Y for Xbox360Gamepad 
		if (Math.abs(leftYstick) < 0.10) {
      leftYstick = 0; // deadband 10% 
    } 
		if (Math.abs(rghtYstick) < 0.10) { 
      rghtYstick = 0; // deadband 10% 
    } 
    // This method will be called once per scheduler run
    ArmPosition();
    ShoulderPosition();
  }
}
