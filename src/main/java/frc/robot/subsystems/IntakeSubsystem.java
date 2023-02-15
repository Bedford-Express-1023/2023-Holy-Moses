// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.schedulers.ConcurrentScheduler;
import com.ctre.phoenix.sensors.CANCoder;
import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import static frc.robot.Constants.Intake.intake_VICTOR_CAN;
import static frc.robot.Constants.Intake.wrist_SPARK_CAN;
import static frc.robot.Constants.Intake.wrist_CANCODER;
import static frc.robot.Constants.Intake.TOF_sensor_CAN;
import static frc.robot.Constants.Intake.maxWristAcceleration;
import static frc.robot.Constants.Intake.maxWristVelocity;;

public class IntakeSubsystem extends SubsystemBase {
  public final VictorSPX intakeMotor = new VictorSPX(intake_VICTOR_CAN);
  public final CANSparkMax wristMotor = new CANSparkMax(wrist_SPARK_CAN, MotorType.kBrushless);
  public final CANCoder wristCANCoder = new CANCoder(wrist_CANCODER);
  public final TimeOfFlight TOFSensor = new TimeOfFlight(TOF_sensor_CAN);

  public double wristCurrentPosition;
  public String currentIntakeCommand;
  public final double intakeSpeed = 0.9;
  public double TOFRange;
  public boolean FullIntakeCheck;

  public final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0,0,0);
  final ProfiledPIDController wristPID = new ProfiledPIDController(0.1, 0.0, 0.0, new Constraints(15, 100));
  /** Creates a new IntakeSubsystem. */
  public IntakeSubsystem() {
    ShuffleboardTab subsystemTab = Shuffleboard.getTab("Subsystems");
    ShuffleboardLayout intakeLayout = subsystemTab.getLayout("Intake", BuiltInLayouts.kList)
                    .withSize(1, 4)
                    .withPosition(0, 0);
    intakeLayout.addNumber("TOF Range", () -> TOFRange);
    intakeLayout.addNumber("Wrist Position", () -> wristCurrentPosition);

    subsystemTab.add("Current Intake Command", currentIntakeCommand, "none");
  }

  public void WristPosition() {
  }

  public void IntakeIn() {
    wristMotor.set(intakeSpeed);
    currentIntakeCommand = "IntakeIn";
  }

  public void IntakeOut() {
    wristMotor.set(-intakeSpeed);
    currentIntakeCommand = "IntakeOut";
  }

  public void IntakeStop() {
    wristMotor.set(0);
    currentIntakeCommand = "IntakeStop";
  }

  public void FullIntakeCheck() {
    if (TOFRange <= 2) {
      FullIntakeCheck = true;
    }
    else {
      FullIntakeCheck = false;
    }
  }

  public void TOFIntake() {
    if (FullIntakeCheck = true) {
      wristMotor.set(intakeSpeed);
    }
    else {
      wristMotor.set(0);
    }
  }

  @Override
  public void periodic() {
    TOFRange = TOFSensor.getRange();
    wristCurrentPosition = wristCANCoder.getAbsolutePosition();
    FullIntakeCheck();
    // This method will be called once per scheduler run
  }
}
