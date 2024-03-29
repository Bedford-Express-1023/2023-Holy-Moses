// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.playingwithfusion.TimeOfFlight;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Intake.*;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonSRX intakeMotor = new TalonSRX(intake_TALON_CAN);
  private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM, intake_SOLENOID_F_CHANNEL, intake_SOLENOID_R_CHANNEL);

  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor.setNeutralMode(NeutralMode.Coast);
    intakeMotor.configOpenloopRamp(0.2);
    intakeMotor.configContinuousCurrentLimit(30);
  }


  /**
   * runs the intake
   * @param speed speed of intake, percent output
   * @param value state of solenoid, kForward kReverse or kOff
   */
  public void intake(double speed) {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void solenoid(Value value) {
    intakeSolenoid.set(value);
  }

  public void intakeStop() {
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (getCurrentCommand() == null) {SmartDashboard.putString("IntakeCommand", "none");}
    else {SmartDashboard.putString("IntakeCommand", getCurrentCommand().getName());}
  }
}
