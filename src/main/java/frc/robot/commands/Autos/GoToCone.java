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
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class GoToCone extends CommandBase {
Swerve s_Swerve;
public final double maxSpeed = 1;
public final static PathPlannerTrajectory driveTrajectory = PathPlanner.loadPath("Go to Cone", 1, 10);
public double timer = 0;

public GoToCone(Swerve s_Swerve) {
      addRequirements(s_Swerve);
        this.s_Swerve = s_Swerve;
        s_Swerve.xController.setConstraints(new Constraints(maxSpeed, 10));
        s_Swerve.yController.setConstraints(new Constraints(maxSpeed, 10));
        s_Swerve.rotaController.setConstraints(new Constraints(1000, 100));
        //driveController = new HolonomicDriveController(s_Swerve.xController, s_Swerve.yController, s_Swerve.rotaController);
  }

  @Override
  public void execute() {
    
    ChassisSpeeds speed = new ChassisSpeeds(
      s_Swerve.xController.calculate(
        s_Swerve.swerveOdometry.getPoseMeters().getX(), 
        driveTrajectory.sample(timer).poseMeters.getX()),
      s_Swerve.yController.calculate(
        s_Swerve.swerveOdometry.getPoseMeters().getY(), 
        driveTrajectory.sample(timer).poseMeters.getY()), 
      //s_Swerve.rotaController.calculate(
        driveTrajectory.sample(timer).poseMeters.getRotation().getRadians()//)
    );
    SmartDashboard.putNumber("SpeedX", speed.vxMetersPerSecond);
    SmartDashboard.putNumber("SpeedY", speed.vyMetersPerSecond);
    SmartDashboard.putNumber("SpeedR", speed.omegaRadiansPerSecond);

    speed = ChassisSpeeds.fromFieldRelativeSpeeds(speed, s_Swerve.getYaw());
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