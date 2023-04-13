package frc.robot;

import java.util.function.BooleanSupplier;

import org.ejml.dense.row.decomposition.svd.SafeSvd_DDRM;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PS4Controller.Axis;
import edu.wpi.first.wpilibj.event.NetworkBooleanEvent;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.*;
import frc.robot.commands.Autos.BottomScore1CubeAnd1Cone;
import frc.robot.commands.Autos.ChargingStation;
import frc.robot.commands.Autos.PathPlannerCommand;
import frc.robot.commands.Drivetrain.DynamicTeleopSwerve;
import frc.robot.commands.Drivetrain.Balance;
import frc.robot.commands.Autos.RightScore3;
//import frc.robot.commands.Drivetrain.AlignToTarget;
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

    private final int armAxis = XboxController.Axis.kLeftY.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(willController, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(willController, XboxController.Button.kX.value);
    private final JoystickButton turnSlow = new JoystickButton(willController, XboxController.Button.kRightBumper.value);
    private final JoystickButton slow = new JoystickButton(willController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton alignToTarget = new JoystickButton(willController, XboxController.Button.kA.value);

    //private final JoystickButton LED = new JoystickButton(oliviaController, XboxController.Button.kStart.value);
    //private final JoystickButton purple = new JoystickButton(oliviaController, XboxController.Button.kBack.value);
    
    private final JoystickButton intake = new JoystickButton(oliviaController, XboxController.Button.kA.value);
    private final JoystickButton outtake = new JoystickButton(oliviaController, XboxController.Button.kX.value);


    private final JoystickButton outtakeFast = new JoystickButton(oliviaController, XboxController.Button.kY.value);

    private final POVButton balance = new POVButton(willController, 0);
    private final POVButton armHigh = new POVButton(oliviaController, 0);
    private final POVButton armMid = new POVButton(oliviaController, 90);
    private final POVButton armLow = new POVButton(oliviaController, 180);
    private final POVButton armFeeder = new POVButton(oliviaController, 270);
    //private final JoystickButton armReverse = new JoystickButton(oliviaController, XboxController.Button.kLeftBumper.value);
    private final JoystickButton armZero = new JoystickButton(oliviaController, XboxController.Button.kB.value);


    /* Subsystems */
    public final Swerve s_Swerve = new Swerve();
    public final Limelight s_Limelight = new Limelight();
    public final IntakeSubsystem s_Intake = new IntakeSubsystem();
    public final Blinkin s_Blinkin = new Blinkin();
    public final ArmSubsystem s_Arm = new ArmSubsystem();
    public final WristSubsystem s_Wrist = new WristSubsystem(s_Arm);


    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        //s_Blinkin.setDefaultCommand(new InstantCommand(() -> s_Blinkin.blue(), s_Blinkin));
        s_Blinkin.setDefaultCommand(
            new LedChange(s_Blinkin, s_Arm)
        );

        s_Arm.setDefaultCommand(
            new ArmToHome(s_Wrist, s_Arm)
                .andThen(new ShoulderToHome(s_Arm)));
        s_Swerve.setDefaultCommand(
            new DynamicTeleopSwerve(
                s_Swerve, s_Arm, s_Limelight,
                () -> -willController.getRawAxis(translationAxis), 
                () -> -willController.getRawAxis(strafeAxis), 
                () -> -willController.getRawAxis(rotationAxis) * .8,
                () -> robotCentric.getAsBoolean(),
                () -> slow.getAsBoolean(),
                () -> turnSlow.getAsBoolean()
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

        autoChooser.addOption("Test Path", new RightScore3(s_Swerve, s_Intake, s_Arm, s_Wrist));
        autoChooser.setDefaultOption("Do Nothing", new WaitCommand(1));
        autoChooser.addOption("Bottom 1 cone and 1 cube", new BottomScore1CubeAnd1Cone(s_Swerve, s_Intake, s_Arm, s_Wrist));
        autoChooser.addOption("Charging Station", new ChargingStation(s_Swerve, s_Intake, s_Arm, s_Wrist));

        SmartDashboard.putData(autoChooser);

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
        balance.whileTrue(new Balance(s_Swerve));
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
        armHigh.whileTrue(new ScoreHigh(s_Arm, s_Wrist));
        armMid.whileTrue(new ScoreMid(s_Wrist, s_Arm, armMid));
        armLow.whileTrue(new ScoreLow(s_Wrist, s_Arm, armLow));
        armFeeder.whileTrue(new ArmFeeder(s_Wrist, s_Arm));
        //armReverse.onTrue(new InstantCommand(() -> s_Arm.shoulderReversed *= -1));
        new Trigger(() -> oliviaController.getLeftTriggerAxis() > 0.5)
            .onTrue(new InstantCommand(() -> s_Arm.shoulderReversed *= -1));
        armZero.onTrue(new InstantCommand(s_Arm::ArmPositionZero));

        intake.whileTrue(new InstantCommand(() -> s_Intake.intake(0.5)))
            .onFalse(new InstantCommand(() -> s_Intake.intakeStop()));
        outtake.whileTrue(new InstantCommand(() -> s_Intake.intake(-0.3)))
            .onFalse(new InstantCommand(() -> s_Intake.intakeStop()));
        outtakeFast.onTrue(new InstantCommand(() -> s_Intake.intake(-0.5)))
            .onFalse(new InstantCommand(() -> s_Intake.intakeStop()));
        new Trigger(() -> oliviaController.getRightTriggerAxis() > 0.5)
            .onTrue(new InstantCommand(() -> s_Intake.solenoid(Value.kForward)))
            .onFalse(new InstantCommand(() -> s_Intake.solenoid(Value.kReverse)));
        //new InstantCommand(() -> s_Arm.ArmManual(oliviaController.getRawAxis(armAxis)));
        //oliviaController.getRawAxis(armAxis)
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    /* .public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new TopScore1CubeAnd1Cone(s_Swerve, s_Intake, s_Arm, s_Wrist);
    }*/
    //hahah hello
}

