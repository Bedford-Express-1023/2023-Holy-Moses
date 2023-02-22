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
  public double limelightXLeft;
  public double limelightYLeft;
  public double limelightVLeft;
  public double limelightAreaLeft;
  public double limelightXRight;
  public double limelightYRight;
  public double limelightVRight;
  public double limelightAreaRight;

  private NetworkTable tableLeft = NetworkTableInstance.getDefault().getTable("limelight-left");
  private NetworkTable tableRight = NetworkTableInstance.getDefault().getTable("limelight-right");

  private NetworkTableEntry txLeft = tableLeft.getEntry("tx");
  private NetworkTableEntry tyLeft = tableLeft.getEntry("ty");
  private NetworkTableEntry tvLeft = tableLeft.getEntry("tv");
  private NetworkTableEntry taLeft = tableLeft.getEntry("ta");

  private NetworkTableEntry txRight = tableRight.getEntry("tx");
  private NetworkTableEntry tyRight = tableRight.getEntry("ty");
  private NetworkTableEntry tvRight = tableRight.getEntry("tv");
  private NetworkTableEntry taRight = tableRight.getEntry("ta");
  
  /** Creates a new Limelight. */
  public Limelight() {
  }

  /** Updates limelightV for Left Limelight
   * pass an int to define which led mode you want
   * @param desiredLEDState 0 is default for pipeline, 1 is off, 2 is blink, 3 is on
   * @return double V; whether limelight has target or not
   */
  public double updateLimelightVLeft(int desiredLEDState){
    tableLeft.getEntry("ledMode").setNumber(desiredLEDState);
    limelightVLeft = tvLeft.getDouble(0.0);
    return limelightVLeft;
  }


    /** Updates limelightX for Left Limelight
   * pass an int to define which led mode you want - 
   * @param desiredLEDState 0 is default for pipeline, 1 is off, 2 is blink, 3 is on
   * @return double X value of target
   */
  public double updateLimelightXLeft(int desiredLEDState){
    tableLeft.getEntry("ledMode").setNumber(desiredLEDState);
    limelightXLeft = txLeft.getDouble(0.0);
    return limelightXLeft;
  }

    /** Updates limelightX for Right Limelight
   * pass an int to define which led mode you want - 
   * @param desiredLEDState 0 is default for pipeline, 1 is off, 2 is blink, 3 is on
   * @return double X value of target
   */
  public double updateLimelightXRight(int desiredLEDState){
    tableRight.getEntry("ledMode").setNumber(desiredLEDState);
    limelightXRight = txRight.getDouble(0.0);
    return limelightXRight;
  }
  
  /**
   * Updates limelightY value for Left Limelight
   * pass an int to define which led mode you want - 
   * @param desiredLEDState 0 is default for pipeline, 1 is off, 2 is blink, 3 is on
   * @return Y value of target
   */
  public double updateLimelightYLeft(int desiredLEDState){
    tableLeft.getEntry("ledMode").setNumber(desiredLEDState);
    limelightYLeft = tyLeft.getDouble(0.0);
    return limelightYLeft;
  }

    /**
   * Updates limelightY value for Right Limelight
   * pass an int to define which led mode you want - 
   * @param desiredLEDState 0 is default for pipeline, 1 is off, 2 is blink, 3 is on
   * @return double Y value of target
   */
  public double updateLimelightYRight(int desiredLEDState){
    tableRight.getEntry("ledMode").setNumber(desiredLEDState);
    limelightYRight = tyRight.getDouble(0.0);
    return limelightYRight;
  }

    /** Updates limelightArea value for Left Limelight
   * pass an int to define which led mode you want 
   * @param desiredLEDState 0 is pipeline default, 1 is off, 2 is blink, 3 is on
   * @return double area of image the target occupies
   */
  public double updateLimelightAreaLeft(int desiredLEDState){
    tableLeft.getEntry("ledMode").setNumber(desiredLEDState);
    limelightAreaLeft = taLeft.getDouble(0.0);
    return limelightAreaLeft;
  }

    /** Updates limelightArea value for Right Limelight
   * pass an int to define which led mode you want 
   * @param desiredLEDState 0 is pipeline default, 1 is off, 2 is blink, 3 is on
   * @return double area of image the target occupies
   */
  public double updateLimelightAreaRight(int desiredLEDState){
    tableRight.getEntry("ledMode").setNumber(desiredLEDState);
    limelightAreaRight = taRight.getDouble(0.0);
    return limelightAreaRight;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("LimelightX LEFT", limelightXLeft);
    SmartDashboard.putNumber("LimelightY LEFT", limelightYLeft);
    SmartDashboard.putNumber("LimelightArea LEFT", limelightAreaLeft);

    SmartDashboard.putNumber("LimelightX RIGHT", limelightXRight);
    SmartDashboard.putNumber("LimelightY RIGHT", limelightYRight);
    SmartDashboard.putNumber("LimelightArea RIGHT", limelightAreaRight);
    // This method will be called once per scheduler run
  }
}
