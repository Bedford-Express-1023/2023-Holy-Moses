package frc.robot;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.commands.Autos.GoToCone;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

    
public class RobotContainer {
    /* Controllers */
    private final Joystick willController = new Joystick(0);
    private final XboxController oliviaController = new XboxController(1);
    private final XboxController testController = new XboxController(4);

    private double slewDouble = 1000.0; //3.0
    private final SlewRateLimiter willSlew = new SlewRateLimiter(slewDouble);


    /* Drive Controls */
    private final int translationAxis = XboxController.Axis.kLeftY.value;
    private final int strafeAxis = XboxController.Axis.kLeftX.value;
    private final int rotationAxis = XboxController.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(willController, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(willController, XboxController.Button.kLeftBumper.value);
    //private final JoystickButton ArmDown = new JoystickButton(testController, XboxController.Button.kX.value);
    //private final JoystickButton ArmUp = new JoystickButton(testController, XboxController.Button.kY.value);
    private final JoystickButton WristTest = new JoystickButton(testController, XboxController.Button.kA.value);
    private final JoystickButton WristTest2 = new JoystickButton(testController, XboxController.Button.kB.value);
    private final JoystickButton ArmHighScore = new JoystickButton(testController, XboxController.Button.kStart.value);
    //private final JoystickButton ArmMiddleScore = new JoystickButton(testController, XboxController.Button.?.value);
    private final JoystickButton ArmLowScore = new JoystickButton(testController, XboxController.Button.kBack.value);


    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ArmSubsystem s_Arm = new ArmSubsystem();
    private final WristSubsystem s_Wrist = new WristSubsystem();
    private final IntakeSubsystem s_Intake = new IntakeSubsystem();

    /* Commands */
    private final ArmStop armStop = new ArmStop(s_Arm);
    private final ArmHighScore armHighScore = new ArmHighScore(s_Arm);
    private final ArmLowScore armLowScore = new ArmLowScore(s_Arm);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Arm.register();
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -willController.getRawAxis(translationAxis), 
                () -> -willController.getRawAxis(strafeAxis), 
                () -> -willController.getRawAxis(rotationAxis),
                () -> robotCentric.getAsBoolean()
            )
        );
        s_Arm.setDefaultCommand(
            new ArmInOut(
                s_Arm, 
                () -> oliviaController.getRawAxis(translationAxis)
            )
        );
        //s_Arm.setDefaultCommand(armStop);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        //ArmUp.onTrue(new InstantCommand(s_Arm::ArmHighScore));
        //ArmDown.onTrue(new InstantCommand(s_Arm::ArmLowScore));
        WristTest.whileTrue(new InstantCommand(() -> s_Wrist.setWrist(-50))/* .alongWith(new InstantCommand(()
        -> s_Intake.Intake(Value.kReverse, -0.5)))*/);
        /*WristTest.whileFalse(new InstantCommand(() -> s_Wrist.setWrist(0))/*.alongWith(
            new InstantCommand(() -> s_Intake.stopIntakeCube())));*/
        WristTest2.whileTrue(new InstantCommand(() -> s_Wrist.setWrist(0)));
        //WristTest2.whileFalse(new InstantCommand(() -> s_Wrist.setWrist(0)));
        ArmLowScore.whileTrue(armLowScore);
        ArmLowScore.whileFalse(armStop);
        ArmHighScore.whileTrue(armHighScore);
        ArmHighScore.whileFalse(armStop);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new GoToCone(s_Swerve);
    }
    //hahah hello
}