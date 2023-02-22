package frc.robot;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.Autos.DoNothing;
import frc.robot.commands.Autos.GoToCone;
import frc.robot.commands.Drivetrain.AlignToTarget;
import frc.robot.commands.Drivetrain.TeleopSwerve;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */

    
public class RobotContainer {
    /* Sendable Choosers */
    public final SendableChooser<Command> autoChooser = new SendableChooser<Command>();
    public final SendableChooser<Command> autoDelay = new SendableChooser<Command>();

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
    private final JoystickButton slowSpeed = new JoystickButton(willController, XboxController.Axis.kLeftTrigger.value);
    private final JoystickButton fastTurn = new JoystickButton(willController, XboxController.Axis.kRightTrigger.value);
    private final JoystickButton alignToTarget = new JoystickButton(willController, XboxController.Button.kRightBumper.value);
    private final JoystickButton ArmDown = new JoystickButton(testController, XboxController.Button.kX.value);
    //private final JoystickButton ArmUp = new JoystickButton(testController, XboxController.Button.kY.value);
    private final JoystickButton WristTest = new JoystickButton(testController, XboxController.Button.kA.value);
    private final JoystickButton WristTest2 = new JoystickButton(testController, XboxController.Button.kB.value);


    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final Limelight s_Limelight = new Limelight();
    //private final ArmSubsystem s_Arm = new ArmSubsystem();


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, s_Limelight, 
                () -> -willController.getRawAxis(translationAxis), 
                () -> -willController.getRawAxis(strafeAxis), 
                () -> -willController.getRawAxis(rotationAxis),
                () -> robotCentric.getAsBoolean(),
                () -> slowSpeed.getAsBoolean(), 
                () -> fastTurn.getAsBoolean()
            )
        );


        autoDelay.setDefaultOption("none", new WaitCommand(0.0));
        autoDelay.addOption("1.0", new WaitCommand(1.0));
        autoDelay.addOption("2.0", new WaitCommand(2.0));
        autoDelay.addOption("3.0", new WaitCommand(3.0));
        autoDelay.addOption("4.0", new WaitCommand(4.0));
        autoDelay.addOption("5.0", new WaitCommand(5.0));
        autoDelay.addOption("6.0", new WaitCommand(6.0));
        autoDelay.addOption("7.0", new WaitCommand(7.0));
        autoDelay.addOption("8.0", new WaitCommand(8.0));
        autoDelay.addOption("9.0", new WaitCommand(9.0));
        autoDelay.addOption("10.0", new WaitCommand(10.0));
        
        SmartDashboard.putData(autoDelay);

        autoChooser.setDefaultOption("Do Nothing", new DoNothing());

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
        alignToTarget.onTrue(new AlignToTarget(s_Swerve, s_Limelight));
        //ArmUp.onTrue(new InstantCommand(s_Arm::ArmHighScore));
        //ArmDown.onTrue(new InstantCommand(s_Arm::ArmLowScore));
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
}
