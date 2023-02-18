// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.schedulers.ConcurrentScheduler;
import com.ctre.phoenix.sensors.CANCoder;
//import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Intake.*;
import static frc.robot.Constants.Intake.maxWristAcceleration;
import static frc.robot.Constants.Intake.maxWristVelocity;;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonSRX intakeMotor = new TalonSRX(intake_TALON_CAN);
  private final DoubleSolenoid intakeSolenoid = new DoubleSolenoid(
        PneumaticsModuleType.CTREPCM, intake_SOLENOID_F_CHANNEL, intake_SOLENOID_R_CHANNEL);

  public final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0,0,0);
  final ProfiledPIDController wristPID = new ProfiledPIDController(0.1, 0.0, 0.0, new Constraints(15, 100));
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    intakeMotor.setNeutralMode(NeutralMode.Coast);
    intakeMotor.configOpenloopRamp(0.2);
    intakeMotor.configContinuousCurrentLimit(20);
    intakeMotor.configPeakCurrentLimit(25);
    /*ShuffleboardTab subsystemTab = Shuffleboard.getTab("Subsystems");
    ShuffleboardLayout intakeLayout = subsystemTab.getLayout("Intake", BuiltInLayouts.kList)
                    .withSize(1, 4)
                    .withPosition(0, 0);
    intakeLayout.addNumber("TOF Range", () -> TOFRange);
    intakeLayout.addNumber("Wrist Position", () -> wristCurrentPosition);

    subsystemTab.add("Current Intake Command", currentIntakeCommand, "none");*/
  }

  public void WristPosition() {
  }


  /**
   * runs the intake
   * @param speed speed of intake, percent output
   * @param value state of solenoid, kForward kReverse or kOff
   */
  public void Intake(double speed, Value value) {
    intakeSolenoid.set(value);
    intakeMotor.set(TalonSRXControlMode.PercentOutput, speed);
  }

  public void intakeStop(){
    intakeMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
