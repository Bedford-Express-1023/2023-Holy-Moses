// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseMotorController;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase {
  TalonFX leftArmMotor = new TalonFX(?);//change ID number
	PigeonIMU pidgeonGyro = new PigeonIMU(0);
	Joystick willController = new Joystick(0);
  Joystick oliviaController = new Joystick(1);

  BaseMotorController rightArmMotor = new TalonFX(?);//change ID number //following leftArmMotor

  
	boolean[] _previous_currentBtns = new boolean[Constants.kNumButtonsPlusOne];
	boolean[] _currentBtns = new boolean[Constants.kNumButtonsPlusOne];

  boolean _firstCall = false; ? //idk what this is
	boolean _state = false; ? //idk what this is
	double targetAngleHigh = 0;
  double targetAngleMedium = 0;
  double targetAngleLow = 0;

//How much smoothing [0,8] to use during MotionMagic 
int smoothing;

  // Creates a new ArmSubsystem. 
  public ArmSubsystem() {
		leftArmMotor.setNeutralMode(NeutralMode.Brake);
		rightArmMotor.setNeutralMode(NeutralMode.Brake);

    leftArmMotor.set(ControlMode.Velocity, smoothing);
    rightArmMotor.set(ControlMode.Velocity, smoothing);

		leftArmMotor.configPeakOutputForward(+1.0);
		leftArmMotor.configPeakOutputReverse(-1.0);
		rightArmMotor.configPeakOutputForward(+1.0);
		rightArmMotor.configPeakOutputReverse(-1.0);

    
		// Set Motion Magic gains in slot0 - see documentation 
		leftArmMotor.selectProfileSlot(Constants.kSlotIdx, Constants.kPIDLoopIdx);
		leftArmMotor.config_kF(Constants.kSlotIdx, Constants.kGains.kF, Constants.kTimeoutMs);
		leftArmMotor.config_kP(Constants.kSlotIdx, Constants.kGains.kP, Constants.kTimeoutMs);
		leftArmMotor.config_kI(Constants.kSlotIdx, Constants.kGains.kI, Constants.kTimeoutMs);
		leftArmMotor.config_kD(Constants.kSlotIdx, Constants.kGains.kD, Constants.kTimeoutMs);
  }

  public void armUp() {
  }

  public void armDown() {
  }

  public void armScoreHigh() {
  }

  public void armScoreMedium() {
  }

  public void armScoreLow() {
  }

  public void armStop() {
  }

  @Override
  public void periodic() {
    double leftYstick = -1.0 * oliviaController.getY(); // left-side Y for Xbox360Gamepad 
		double rghtYstick = -1.0 * oliviaController.getRawAxis(?); // right-side Y for Xbox360Gamepad 
		if (Math.abs(leftYstick) < 0.10) {
      leftYstick = 0; // deadband 10% 
    } 
		if (Math.abs(rghtYstick) < 0.10) { 
      rghtYstick = 0; // deadband 10% 
    } 
    // This method will be called once per scheduler run
  }
} */
