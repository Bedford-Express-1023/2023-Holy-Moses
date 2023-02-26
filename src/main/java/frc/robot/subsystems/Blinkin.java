// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Blinkin extends SubsystemBase {
  private final Spark m_Spark = new Spark(Constants.BLINKIN);
  /** Creates a new blinkin. */
  public Blinkin() {}

  public void yellow(){
    m_Spark.set(0.69);
  }

  public void purple(){
    m_Spark.set(0.91);
  }

  public void blue (){
    m_Spark.set(-0.23);
  }


public void limitSwitchColor() {
}



  @Override
  public void periodic() {
   
   }
    // This method will be called once per scheduler run
  }

