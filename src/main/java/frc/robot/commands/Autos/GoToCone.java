// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Swerve;

public class GoToCone extends PPSwerveControllerCommand {
 Swerve s_Swerve;
  private final static PathPlannerTrajectory driveTrajectory = PathPlanner.loadPath("Go to Cone", 100, 8);
  private double timer = 0;

public GoToCone(Swerve s_Swerve) {
      super(
        driveTrajectory, 
        s_Swerve::getPose, 
        s_Swerve.xController, 
        s_Swerve.yController, 
        s_Swerve.rotaController, 
        (x) -> s_Swerve.drive(new Translation2d(x.vxMetersPerSecond, x.vyMetersPerSecond), x.omegaRadiansPerSecond, false, false ), 
        true,
        s_Swerve);

  }
 // @Override
  /*public void execute() {
  ChassisSpeeds speed = s_Swerve.driveController.calculate(s_Swerve.odometry.getPoseMeters(), driveTrajectory.sample(timer), driveTrajectory.sample(timer).poseMeters.getRotation());
s_Swerve.drive(new Translation2d(speed.vxMetersPerSecond, speed.vyMetersPerSecond), speed.omegaRadiansPerSecond, false, false);
  timer += 0.02;
  }

  @Override
  public boolean isFinished() {
      return false;//driveTrajectory.getTotalTimeSeconds() <= timer - 0.2;
  }
  @Override
  public void end(boolean interrupted) {
      // Stop the drivetrain
      s_Swerve.drive(new Translation2d(0.0, 0.0), 0, false, false);
      timer = 0;
  }*/
}