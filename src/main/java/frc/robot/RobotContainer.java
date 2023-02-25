package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Constants.Intake;
import frc.robot.commands.*;
import frc.robot.commands.Autos.TopScore1CubeAnd1Cone;
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
    private final JoystickButton turnSLow = new JoystickButton(willController, XboxController.Button.kRightBumper.value);
    private final JoystickButton slow = new JoystickButton(willController, XboxController.Button.kX.value);

    private final JoystickButton intakeDropCube = new JoystickButton(oliviaController, XboxController.Button.kX.value);
    private final JoystickButton intakeDropCone = new JoystickButton(oliviaController, XboxController.Button.kY.value);
    private final JoystickButton intakeCube = new JoystickButton(oliviaController, XboxController.Button.kA.value);
    private final JoystickButton intakeCone = new JoystickButton(oliviaController, XboxController.Button.kB.value);
    private final JoystickButton yellow = new JoystickButton(oliviaController, XboxController.Button.kStart.value);
    private final JoystickButton purple = new JoystickButton(oliviaController, XboxController.Button.kBack.value);

    private final POVButton ArmHigh = new POVButton(oliviaController, 0);
    private final POVButton ArmMid = new POVButton(oliviaController, 90);
    private final POVButton ArmLow = new POVButton(oliviaController, 180);


    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final ArmSubsystem s_Arm = new ArmSubsystem();
    private final IntakeSubsystem s_Intake = new IntakeSubsystem();
    private final WristSubsystem s_Wrist = new WristSubsystem();
    private final Blinkin s_Blinkin = new Blinkin();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -willController.getRawAxis(translationAxis), 
                () -> -willController.getRawAxis(strafeAxis), 
                () -> -willController.getRawAxis(rotationAxis) * .8,
                () -> robotCentric.getAsBoolean(),
                () -> turnSLow.getAsBoolean(),
                () -> slow.getAsBoolean()
                
            )
        );

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
        ArmHigh.onTrue(new ScoreHigh(s_Wrist, s_Arm, ArmHigh));
        ArmMid.onTrue(new ScoreMid(s_Wrist, s_Arm, ArmMid));
        ArmLow.onTrue(new ScoreLow(s_Wrist, s_Arm, ArmLow));

        intakeCube.whileTrue(new InstantCommand(() -> s_Intake.intake(0.5, Value.kForward)))
            .onFalse(new InstantCommand(() -> s_Intake.intakeStop()));
        intakeDropCube.onTrue(new InstantCommand(() -> s_Intake.intake(-0.5, Value.kForward)))
            .onFalse(new InstantCommand(() -> s_Intake.intakeStop()));
        intakeDropCone.onTrue(new InstantCommand(() -> s_Intake.intake(0.0, Value.kForward)))
            .onFalse(new InstantCommand(() -> s_Intake.intakeStop()));;
        intakeCone.whileTrue(new InstantCommand(() -> s_Intake.intake(0.5, Value.kReverse)))
            .onFalse(new InstantCommand(() -> s_Intake.intakeStop()));
        yellow.whileTrue(new InstantCommand(() -> s_Blinkin.yellow()))
            .whileFalse(new InstantCommand(() -> s_Blinkin.blue()));
        purple.whileTrue(new InstantCommand(() -> s_Blinkin.purple()))
            .whileFalse(new InstantCommand(() -> s_Blinkin.blue()));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new TopScore1CubeAnd1Cone(s_Swerve);
    }
}
