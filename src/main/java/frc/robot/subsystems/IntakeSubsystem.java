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

public class IntakeSubsystem extends SubsystemBase {
  public final VictorSPX intakeMotor = new VictorSPX(?);
  public final CANSparkMax wristMotor = new CANSparkMax(?, MotorType.kBrushless);
  public final CANCoder wristCANCoder = new CANCoder(?);
  public final TimeOfFlight TOFSensor = new TimeOfFlight(1);

  public double wristCurrentPosition;
  public String currentIntakeCommand;
  public final double intakeSpeed = 0.9;
  public final double maxWristVelocity = 15;
  public final double maxWristAcceleration = 15;
  public final double TOFRange;
  public final boolean FullIntakeCheck;

  public final SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(?, ?);
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

  public boolean FullIntakeCheck() {
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
    // This method will be called once per scheduler run
  }
}
