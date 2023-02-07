// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class GoToCone extends PPSwerveControllerCommand {
 Swerve s_Swerve;
  public final static PathPlannerTrajectory driveTrajectory = PathPlanner.loadPath("Go to Cone", 5, 5);
  public HolonomicDriveController driveController;
  public double timer = 0;

public GoToCone(Swerve s_Swerve) {
      super(
        driveTrajectory, 
        s_Swerve::getPose,
        s_Swerve.kinematics,
        s_Swerve.xController, 
        s_Swerve.yController, 
        new PIDController(s_Swerve.rotaController.getP(), s_Swerve.rotaController.getI(), s_Swerve.rotaController.getD()), 
        (states) -> s_Swerve.setModuleStates(states), 
        false,
        s_Swerve);
        this.s_Swerve = s_Swerve;
        driveController = new HolonomicDriveController(s_Swerve.xController, s_Swerve.yController, s_Swerve.rotaController);
  }
 @Override
public void execute() {
  ChassisSpeeds speed = s_Swerve.xController.calculate(s_Swerve.swerveOdometry.getPoseMeters(), driveTrajectory.sample(timer), driveTrajectory.sample(timer).poseMeters.getRotation());
  SwerveModuleState[] states = s_Swerve.kinematics.toSwerveModuleStates(speed);
  s_Swerve.setModuleStates(states);
  //s_Swerve.drive(new Translation2d(speed.vxMetersPerSecond, speed.vyMetersPerSecond).times(Constants.Swerve.maxSpeed), speed.omegaRadiansPerSecond * Constants.Swerve.maxAngularVelocity, false, false);
  timer += 0.02;
  }

  @Override
  public boolean isFinished() {
      return driveTrajectory.getTotalTimeSeconds() <= timer - 0.2;
  }
  @Override
  public void end(boolean interrupted) {
      // Stop the drivetrain
      s_Swerve.drive(new Translation2d(0.0, 0.0), 0, false, false);
      timer = 0;
  }
}