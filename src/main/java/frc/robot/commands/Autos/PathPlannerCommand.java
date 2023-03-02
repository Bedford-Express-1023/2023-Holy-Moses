// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
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

public PathPlannerCommand(Swerve s_Swerve, double maxSpeed, String pathName) {
      addRequirements(s_Swerve);
        this.s_Swerve = s_Swerve;
        this.maxSpeed = maxSpeed;
        this.driveTrajectory = PathPlannerTrajectory.transformTrajectoryForAlliance(PathPlanner.loadPath(pathName, 3, 10), DriverStation.getAlliance());
        s_Swerve.xController.setConstraints(new Constraints(maxSpeed, 10));
        s_Swerve.yController.setConstraints(new Constraints(maxSpeed, 10));
        s_Swerve.rotaController.setConstraints(new Constraints(1000, 100));
        //driveController = new HolonomicDriveController(s_Swerve.xController, s_Swerve.yController, s_Swerve.rotaController);
  }

  @Override
  public void initialize() {
      //s_Swerve.resetOdometry(new Pose2d(driveTrajectory.sample(0).poseMeters.getTranslation(), ((PathPlannerState)driveTrajectory.sample(0)).holonomicRotation));
      timer.start();
      s_Swerve.resetOdometry(new Pose2d());
  }

  @Override
  public void execute() {
    PathPlannerState state = (PathPlannerState) driveTrajectory.sample(timer.get());
    state.poseMeters = new Pose2d(
      state.poseMeters.getX() - driveTrajectory.getInitialPose().getX(),
      state.poseMeters.getY() - driveTrajectory.getInitialPose().getY(), 
      state.poseMeters.getRotation().minus(driveTrajectory.getInitialPose().getRotation()));
    ChassisSpeeds speed = new ChassisSpeeds(
      s_Swerve.xController.calculate(
        s_Swerve.swerveOdometry.getPoseMeters().getX(), 
        state.poseMeters.getX()),
      s_Swerve.yController.calculate(
        s_Swerve.swerveOdometry.getPoseMeters().getY(), 
        state.poseMeters.getY()), 
      s_Swerve.rotaController.calculate(
        state.holonomicRotation.getDegrees())
        //state.holonomicRotation.getRadians())
    );
    SmartDashboard.putNumber("SpeedX", speed.vxMetersPerSecond);
    SmartDashboard.putNumber("SpeedY", speed.vyMetersPerSecond);
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
      s_Swerve.drive(new Translation2d(0.0, 0.0), 0, false, false);
      timer.reset();
  }
}