// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  public double limelightX;
  public double limelightY;
  public double limelightArea;
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  private NetworkTableEntry tx = table.getEntry("tx");
  private NetworkTableEntry ty = table.getEntry("ty");
  private NetworkTableEntry ta = table.getEntry("ta");
  
  /** Creates a new Limelight. */
  public Limelight() {
  }

    /**
   * pass an int to define which led mode you want - 
   * 0 is default for pipeline, 1 is off, 2 is blink, 3 is on
   * @return
   */
  public double updateLimelightX(int desiredLEDState){
    table.getEntry("ledMode").setNumber(desiredLEDState);
    limelightX = tx.getDouble(0.0);
    return limelightX;
  }
  
  /**
   * pass an int to define which led mode you want - 
   * 0 is default for pipeline, 1 is off, 2 is blink, 3 is on
   * @return
   */
  public double updateLimelightY(int desiredLEDState){
    table.getEntry("ledMode").setNumber(desiredLEDState);
    limelightY = ty.getDouble(0.0);
    return limelightY;
  }

    /**
   * pass an int to define which led mode you want - 
   * 0 is default for pipeline, 1 is off, 2 is blink, 3 is on
   * @return
   */
  public double updateLimelightArea(int desiredLEDState){
    table.getEntry("ledMode").setNumber(desiredLEDState);
    limelightArea = ta.getDouble(0.0);
    return limelightArea;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LimelightX", limelightX);
    SmartDashboard.putNumber("LimelightY", limelightY);
    SmartDashboard.putNumber("LimelightArea", limelightArea);
    // This method will be called once per scheduler run
  }
}
