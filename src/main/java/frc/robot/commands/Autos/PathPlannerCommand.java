// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class PathPlannerCommand extends CommandBase {
Swerve s_Swerve;
public double maxSpeed = 3;
public PathPlannerTrajectory driveTrajectory;
Timer timer = new Timer();
public Boolean autoStart = false;

public PathPlannerCommand(Swerve s_Swerve, double maxSpeed, String pathName, Boolean autoStart) {
      addRequirements(s_Swerve);
        this.s_Swerve = s_Swerve;
        this.maxSpeed = maxSpeed;
        this.autoStart = autoStart;
        this.driveTrajectory = PathPlanner.loadPath(pathName, maxSpeed, 10);
  }

  public PathPlannerCommand(Swerve s_Swerve, double maxSpeed, String pathName) {
    this(s_Swerve, maxSpeed, pathName, false);
  }

  @Override
  public void initialize() {
    driveTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(driveTrajectory, DriverStation.getAlliance());
      timer.start();
      if (autoStart) {
        s_Swerve.gyro.setYaw(driveTrajectory.getInitialHolonomicPose().getRotation().getDegrees());
        s_Swerve.resetOdometry(driveTrajectory.getInitialHolonomicPose());
      }
  }

  @Override
  public void execute() {
    PathPlannerState state = (PathPlannerState) driveTrajectory.sample(timer.get());
    Pose2d targetPose = new Pose2d(
      state.poseMeters.getX() - driveTrajectory.getInitialPose().getX(),
      state.poseMeters.getY() - driveTrajectory.getInitialPose().getY(), 
      state.holonomicRotation.minus(driveTrajectory.getInitialHolonomicPose().getRotation()));
    Pose2d robotPose = new Pose2d(
      s_Swerve.getPose().getX() - driveTrajectory.getInitialHolonomicPose().getX(),
      s_Swerve.getPose().getY() - driveTrajectory.getInitialHolonomicPose().getY(),
      s_Swerve.getPose().getRotation().minus(driveTrajectory.getInitialHolonomicPose().getRotation()));
    ChassisSpeeds speed = new ChassisSpeeds(
      MathUtil.clamp(s_Swerve.xController.calculate(
        robotPose.getX(),
        targetPose.getX()), -maxSpeed, maxSpeed),
      MathUtil.clamp(s_Swerve.yController.calculate(
        robotPose.getY(), 
        targetPose.getY()), -maxSpeed, maxSpeed),
      s_Swerve.rotaController.calculate(
        robotPose.getRotation().getRadians(),
        targetPose.getRotation().getRadians())
    );
    SmartDashboard.putNumber("CurrentRotation", robotPose.getRotation().getRadians());
    SmartDashboard.putNumber("TargetRotation", targetPose.getRotation().getRadians());
    SmartDashboard.putNumber("SpeedR", speed.omegaRadiansPerSecond);

    speed = ChassisSpeeds.fromFieldRelativeSpeeds(speed, s_Swerve.getYaw());
    SwerveModuleState[] states = s_Swerve.kinematics.toSwerveModuleStates(speed);
    s_Swerve.setModuleStates(states);
    //s_Swerve.drive(new Translation2d(speed.vxMetersPerSecond, speed.vyMetersPerSecond).times(Constants.Swerve.maxSpeed), speed.omegaRadiansPerSecond * Constants.Swerve.maxAngularVelocity, false, false);
  }

  @Override
  public boolean isFinished() {
      return driveTrajectory.getTotalTimeSeconds() <= timer.get() - 0.2;
  }

  @Override
  public void end(boolean interrupted) {
      // Stop the drivetrain
      s_Swerve.drive(new Translation2d(0.0, 0.0), 0, true, false);
      timer.reset();
  }
}