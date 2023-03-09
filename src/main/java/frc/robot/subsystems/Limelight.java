// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public final class Limelight {
  public final static int Left = 0;
  public final static int Right = 1;
  public final Pose3d LimelightPose = new Pose3d(2.23, 18.16, 15, new Rotation3d(0, 0, 0)).times(.0254);

  public final static NetworkTable limeLightLeft = NetworkTableInstance.getDefault().getTable("limelight-left");
  public final static NetworkTable limeLightRight = NetworkTableInstance.getDefault().getTable("limelight-right");

  public static int getClosest() {
    if (ta(Left) <= ta(Right)) {
      return Left;
    }
    return Right;
  }

  /**
   * @return if any limelight has a valid target
   */
  public static Boolean tv() {
    return 
      NetworkTableInstance.getDefault().getTable("limelight-left").getEntry("tv").getDouble(0) == 1 ||
      NetworkTableInstance.getDefault().getTable("limelight-right").getEntry("tv").getDouble(0) == 1;
  }

  /**
   * 
   * @param side which limelight, Limelight.Left or Limelight.Right, to use.
   * @return if that specific limelight has a valid target
   */
  public static Boolean tv(int side) {
    return side == Left ? 
      limeLightLeft.getEntry("tv").getDouble(0) == 1:
      limeLightRight.getEntry("tv").getDouble(0) == 1;
      
  }

  /**
   * @param side which limelight, Limelight.Left or Limelight.Right, to use.
   * @return area of the target 0-100% 
   */
  public static double ta(int side) {
    return (side == 1) ? 
    limeLightRight.getEntry("ta").getDouble(0) : 
    limeLightLeft.getEntry("ta").getDouble(0);
  }

  /**
   * @param side which limelight, Limelight.Left or Limelight.Right, to use.
   * @return the horizontal angle to the target
   */
  public static double tx(int side) {
    return (side == 1) ? 
    limeLightRight.getEntry("tx").getDouble(0) : 
    limeLightLeft.getEntry("tx").getDouble(0);
  }

  /**
   * @param side which limelight, Limelight.Left or Limelight.Right, to use.
   * @return the vertical angle to the target
   */
  public static double ty(int side) {
    return (side == 1) ? 
    limeLightRight.getEntry("ty").getDouble(0) : 
    limeLightLeft.getEntry("ty").getDouble(0);
  }

  /**
   * @param side which limelight, Limelight.Left or Limelight.Right, to use.
   * @return the vertical angle to the target
   */
  public static Pose3d botPose(int side) {
    double[] left = {0,0,0,0,0,0};
    double[] right = {0,0,0,0,0,0};
    if (limeLightLeft.getEntry("botpose").getDoubleArray(new double[] {0,0,0,0,0,0}).length > 1)
      {left = limeLightLeft.getEntry("botpose").getDoubleArray(new double[] {0,0,0,0,0,0});};
    if (limeLightRight.getEntry("botpose").getDoubleArray(new double[] {0,0,0,0,0,0}).length > 1)
      {right = limeLightRight.getEntry("botpose").getDoubleArray(new double[] {0,0,0,0,0,0});};
      return (side == 1) ? 
      new Pose3d(
        right[0],
        right[1],
        right[2],
        new Rotation3d(
          Math.toRadians(right[3]),
          Math.toRadians(right[4]),
          Math.toRadians(right[5])
      )): 
      new Pose3d(
        left[0],
        left[1],
        left[2],
        new Rotation3d(
          Math.toRadians(left[3]),
          Math.toRadians(left[4]),
          Math.toRadians(left[5])
      ));
  }
}
