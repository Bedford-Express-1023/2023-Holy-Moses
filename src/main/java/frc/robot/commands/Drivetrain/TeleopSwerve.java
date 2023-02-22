package frc.robot.commands.Drivetrain;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.hal.DriverStationJNI;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;


public class TeleopSwerve extends CommandBase {    
    private Swerve s_Swerve;    
    private DoubleSupplier translationSup;
    private DoubleSupplier strafeSup;
    private DoubleSupplier rotationSup;
    private BooleanSupplier robotCentricSup;
    private BooleanSupplier slowSpeedSup;
    private BooleanSupplier fastTurnSup;

    public TeleopSwerve(Swerve s_Swerve, DoubleSupplier translationSup, DoubleSupplier strafeSup, DoubleSupplier rotationSup, BooleanSupplier robotCentricSup, BooleanSupplier slowSpeedSup, BooleanSupplier fastTurnSup) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        this.translationSup = translationSup;
        this.strafeSup = strafeSup;
        this.rotationSup = rotationSup;
        this.robotCentricSup = robotCentricSup;
        this.slowSpeedSup = slowSpeedSup;
        this.fastTurnSup = fastTurnSup;
    }

    @Override
    public void execute() {
        /* Get Values, Deadband*/
        double translationVal;
        double strafeVal;
        double rotationVal;
            if (slowSpeedSup.getAsBoolean() == true){
                translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband) * Constants.Swerve.slowSpeedMultiplier;
                strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband) * Constants.Swerve.slowSpeedMultiplier;
                rotationVal = MathUtil.applyDeadband(rotationSup.getAsDouble(), Constants.stickDeadband);
            }
            else {
                translationVal = MathUtil.applyDeadband(translationSup.getAsDouble(), Constants.stickDeadband);
                strafeVal = MathUtil.applyDeadband(strafeSup.getAsDouble(), Constants.stickDeadband);
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