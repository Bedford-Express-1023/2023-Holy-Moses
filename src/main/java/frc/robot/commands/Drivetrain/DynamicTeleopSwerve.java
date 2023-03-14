package frc.robot.commands.Drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class DynamicTeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private ArmSubsystem s_Arm;
    private Limelight s_Limelight;
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier slowSpeedSup;
    private BooleanSupplier fastTurnSup;

    public DynamicTeleopSwerve(Swerve s_Swerve, ArmSubsystem s_Arm, Limelight s_Limelight, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier slowSpeedSup, BooleanSupplier fastTurnSup) {
        this.s_Swerve = s_Swerve;
        this.s_Arm = s_Arm;
        addRequirements(s_Swerve, s_Arm);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.slowSpeedSup = slowSpeedSup;
        this.fastTurnSup = fastTurnSup;
        this.s_Limelight = s_Limelight;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal = 0;
        double strafeVal = 0;
        double rotationVal = 0;

        double armPosition = s_Arm.armPosition;
        double dynamicDriveSpeed = (100 - (1e-8) * armPosition * armPosition)/100;

            if (slowSpeedSup.getAsBoolean() == true){
                translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband) * Constants.Swerve.slowSpeedMultiplier;
                strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband) * Constants.Swerve.slowSpeedMultiplier;
                rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
            }
            else {
                translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband) * dynamicDriveSpeed;
                strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband) * dynamicDriveSpeed;
                rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
            }
        
        /* Drive */
        if (fastTurnSup.getAsBoolean() == true){
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed),
                rotationVal * Constants.Swerve.fastTurnMultiplier,
                !robotCentricSup.getAsBoolean(),
                true);
            }

        else if (fastTurnSup.getAsBoolean() == false){
            s_Swerve.drive(
                new Translation2d(translationVal, strafeVal).times(Constants.Swerve.maxSpeed), 
                rotationVal * Constants.Swerve.maxAngularVelocity, 
                !robotCentricSup.getAsBoolean(), 
                true);
        }


    }
}