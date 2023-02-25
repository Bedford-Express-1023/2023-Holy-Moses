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
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.RotationalFeedForward;
import frc.robot.Constants;
import static frc.robot.Constants.Arm.maxShoulderVelocity;
import static frc.robot.Constants.Arm.maxShoulderAcceleration;
import static frc.robot.Constants.Arm.maxArmVelocity;
import static frc.robot.Constants.Arm.maxArmAcceleration;
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

  public double armPosition = 0;
  final PIDController armPID = new PIDController(0.00007, 0.0, 0);
  public double shoulderPosition = 0;
  public double shoulderPositionOverride = 0;
  public int shoulderReversed = -1;

  public final double shoulderGravity = .075;
  public final RotationalFeedForward feedForward = new RotationalFeedForward(0,1,0, shoulderGravity);
  final PIDController shoulderPositionPID = new PIDController(.015, 0.0, 0);

	final public double shoulderTargetAngleHigh = 15;
  final public double shoulderTargetAngleMiddle = 30;
  final public double shoulderTargetAngleLow = 105;

  public final double armTargetPositionHigh = -50000;
  public final double armTargetPositionMiddle = -25000;
  public final double armTargetPositionLow = 0;

  final double armGravity = -0.225;

  public String currentShoulderCommand = "";  
  public String currentArmCommand = "";
  public final DigitalInput limitSwitch = new DigitalInput(1);
  public final Counter counter = new Counter(limitSwitch);

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
		rearShoulderMotor.setNeutralMode(NeutralMode.Brake);
		frontShoulderMotor.setNeutralMode(NeutralMode.Brake);
    frontShoulderMotor.follow(rearShoulderMotor);
    armMotor.setNeutralMode(NeutralMode.Brake);

		armMotor.configPeakOutputForward(0.5);
		armMotor.configPeakOutputReverse(-0.5);

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
    //rotationLayout.add("Current Arm Command", currentArmCommand, "none");
  }

  /**
   * sets the shoulder position to the stored position
   * probably use the other overload though
   */
  public void ArmPosition() {
    double vSetpoint = armPID.calculate(armMotor.getSelectedSensorPosition(), armPosition);
    SmartDashboard.putNumber("Extension Setpoint", vSetpoint);
    SmartDashboard.putNumber("Target Position", armPosition);
    armMotor.set(ControlMode.PercentOutput,
    MathUtil.clamp(armGravity +
        vSetpoint,
        -Math.abs(-feedForward.calculate(MathUtil.clamp(Math.abs(vSetpoint), -maxArmVelocity, maxArmVelocity), maxArmAcceleration)),
        Math.abs(feedForward.calculate(MathUtil.clamp(Math.abs(vSetpoint), -maxArmVelocity, maxArmVelocity), maxArmAcceleration)))); //calculates max power output so as not to go above max velocity and max accel
  }

  public void ArmPosition(double position) {
    armPosition = position;
  }

  public void ArmPercent(double percent){
    armMotor.set(TalonFXControlMode.PercentOutput, percent);
  }
  
  public void ArmPositionZero() {
    armMotor.setSelectedSensorPosition(0);
  }
  public void ShoulderPosition() {
    double vSetpoint = shoulderPositionPID.calculate(shoulderCANCoder.getAbsolutePosition(), shoulderPosition);

    SmartDashboard.putNumber("gravity", shoulderGravity * -Math.sin(shoulderCANCoder.getAbsolutePosition()));
    SmartDashboard.putNumber("Shoulder Setpoint", vSetpoint);
    rearShoulderMotor.set(ControlMode.PercentOutput, shoulderGravity * -Math.sin(shoulderCANCoder.getAbsolutePosition() * Math.PI/180) + 
      MathUtil.clamp(
        vSetpoint,
        -Math.abs(-feedForward.calculate(MathUtil.clamp(Math.abs(vSetpoint), -maxShoulderVelocity, maxShoulderVelocity), -maxShoulderAcceleration)),
        Math.abs(feedForward.calculate(MathUtil.clamp(Math.abs(vSetpoint), -maxShoulderVelocity, maxShoulderVelocity), maxShoulderAcceleration)))); //calculates max power output so as not to go above max velocity and max accel //calculates max power output so as not to go above max velocity and max accel
  }

  public void ShoulderPosition(double position) {
    shoulderPosition = position;
  }
  
  @Override
  public void periodic() {
    SmartDashboard.putNumber("ExtensionPosition", armMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("ExtensionVelocity", armMotor.getMotorOutputVoltage());
    SmartDashboard.putNumber("ShoulderTarget", shoulderPosition);
    SmartDashboard.putNumber("ShoulderPosition", shoulderCANCoder.getAbsolutePosition());
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
    //ShoulderPosition(0);
    ShoulderPosition();
  }
} 
