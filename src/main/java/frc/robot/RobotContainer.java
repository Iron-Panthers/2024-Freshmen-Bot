// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.Config;
import frc.robot.Constants.Drive;
import frc.robot.Constants.Drive.Setpoints;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.RotateAngleDriveCommand;
import frc.robot.commands.RotateVectorDriveCommand;
import frc.robot.commands.RotateVelocityDriveCommand;
import frc.robot.commands.ShootCommand;
// import frc.robot.commands.SpitCommand;
import frc.robot.commands.VibrateHIDCommand;
import frc.robot.subsystems.DrivebaseSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.util.ControllerUtil;
import frc.util.Layer;
import frc.util.MacUtil;
import frc.util.Util;
import java.util.Map;
import java.util.function.DoubleSupplier;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...

    private final DrivebaseSubsystem drivebaseSubsystem = new DrivebaseSubsystem();
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();

    /** controller 1 */
    private final CommandXboxController jacob = new CommandXboxController(1);
    /** controller 1 layer */
    private final Layer jacobLayer = new Layer(jacob.rightBumper());
    /** controller 0 */
    private final CommandXboxController anthony = new CommandXboxController(0);
    /** controller 0 layer */
    // private final Layer anthonyLayer = new Layer(anthony.rightBumper());

    /** the sendable chooser to select which auto to run. */
    private final SendableChooser<Command> autoSelector;

    private GenericEntry autoDelay;

    private Pose2d desiredPose;

    private final ShuffleboardTab driverView = Shuffleboard.getTab("DriverView");

  /* drive joystick "y" is passed to x because controller is inverted */
  private final DoubleSupplier translationXSupplier =
      () -> (-axisScaler(anthony.getLeftX(), anthony.getLeftY())*anthony.getLeftY() * Drive.MAX_VELOCITY_METERS_PER_SECOND);
  private final DoubleSupplier translationYSupplier =
      () -> (-axisScaler(anthony.getLeftX(), anthony.getLeftY())*anthony.getLeftX() * Drive.MAX_VELOCITY_METERS_PER_SECOND);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation
        drivebaseSubsystem.setDefaultCommand(
                new DefaultDriveCommand(
                        drivebaseSubsystem,
                        translationXSupplier,
                        translationYSupplier,
                        // anthony.rightBumper(),
                        anthony.leftBumper()));

        // pivotSubsystem.setDefaultCommand(
        // new PivotManualCommand(pivotSubsystem, () -> -jacob.getLeftY()));

        // Configure the button bindings
        configureButtonBindings();

        autoSelector = AutoBuilder.buildAutoChooser();

        SmartDashboard.putBoolean("is comp bot", MacUtil.IS_COMP_BOT);
        SmartDashboard.putBoolean("show debug data", Config.SHOW_SHUFFLEBOARD_DEBUG_DATA);
        SmartDashboard.putBoolean("don't init swerve modules", Config.DISABLE_SWERVE_INIT);

        desiredPose = new Pose2d();
        SmartDashboard.putString(
                "desired pose",
                String.format(
                        "(%2f %2f %2f)",
                        desiredPose.getX(), desiredPose.getY(), desiredPose.getRotation().getDegrees()));

        if (Config.SHOW_SHUFFLEBOARD_DEBUG_DATA) {
            driverView.addDouble("Drivebase Angle Error", () -> drivebaseSubsystem.getAngularError());
        }

        // Create and put autonomous selector to dashboard
        setupAutonomousCommands();
    }

    /**
     * Use this method to do things as the drivers gain control of the robot. We use
     * it to vibrate the
     * driver b controller to notice accidental swaps.
     *
     * <p>
     * Please use this very, very sparingly. It doesn't exist by default for good
     * reason.
     */
    public void containerTeleopInit() {
        // runs when teleop happens
        CommandScheduler.getInstance().schedule(new VibrateHIDCommand(jacob.getHID(), 5, .5));
        // vibrate controller at 27 seconds left
        CommandScheduler.getInstance()
                .schedule(
                        new WaitCommand(108)
                                .andThen(
                                        new ParallelCommandGroup(
                                                new VibrateHIDCommand(anthony.getHID(), 3, 0.4),
                                                new VibrateHIDCommand(jacob.getHID(), 3, 0.4))));
    }

    /**
     * Use this method to do things as soon as the robot starts being used. We use
     * it to stop doing
     * things that could be harmful or undesirable during game play--rebooting the
     * network switch is a
     * good example. Subsystems need to be explicitly wired up to this method.
     *
     * <p>
     * Depending on which mode the robot is enabled in, this will either be called
     * before auto or
     * before teleop, whichever is first.
     *
     * <p>
     * Please use this very, very sparingly. It doesn't exist by default for good
     * reason.
     */
    public void containerMatchStarting() {
        // runs when the match starts
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // vibrate jacob controller when in layer
        jacobLayer.whenChanged(
                (enabled) -> {
                    final double power = enabled ? .1 : 0;
                    jacob.getHID().setRumble(RumbleType.kLeftRumble, power);
                    jacob.getHID().setRumble(RumbleType.kRightRumble, power);
                });

        anthony
                .start()
                .onTrue(new InstantCommand(drivebaseSubsystem::zeroGyroscope, drivebaseSubsystem));

        jacob
                .start()
                .onTrue(new InstantCommand(drivebaseSubsystem::smartZeroGyroscope, drivebaseSubsystem));

        jacob
                .a()
                .onTrue(
                        new RotateAngleDriveCommand(
                                drivebaseSubsystem,
                                translationXSupplier,
                                translationYSupplier,
                                DriverStation.getAlliance().get().equals(Alliance.Red) ? -40 : 40));
        jacob
                .leftBumper()
                .whileTrue(
                        new ShootCommand(
                                shooterSubsystem, false));
        jacob
                .rightBumper()
                .whileTrue(
                        new IntakeCommand(
                                shooterSubsystem,false));
        jacob
                .rightTrigger()
                .whileTrue(
                        new IntakeCommand(
                                shooterSubsystem,true));
        jacob
                .leftTrigger()
                .whileTrue(
                        new ShootCommand(
                                shooterSubsystem, true));

        // SOURCE
        anthony
                .y()
                .onTrue(
                        new RotateAngleDriveCommand(
                                drivebaseSubsystem,
                                translationXSupplier,
                                translationYSupplier,
                                DriverStation.getAlliance().get().equals(Alliance.Red)
                                        ? -Setpoints.SOURCE_DEGREES
                                        : Setpoints.SOURCE_DEGREES));

        // SPEAKER FROM STAGE
        anthony
                .b()
                .onTrue(
                        new RotateAngleDriveCommand(
                                drivebaseSubsystem,
                                translationXSupplier,
                                translationYSupplier,
                                DriverStation.getAlliance().get().equals(Alliance.Red)
                                        ? -Setpoints.SPEAKER_DEGREES
                                        : Setpoints.SPEAKER_DEGREES));

        // AMP
        jacob
                .b()
                .onTrue(
                        new RotateAngleDriveCommand(
                                drivebaseSubsystem,
                                translationXSupplier,
                                translationYSupplier,
                                DriverStation.getAlliance().get().equals(Alliance.Red) ? -90 : 90));

        /*
         * jacob
         * .a()
         * .onTrue(
         * new RotateAngleDriveCommand(
         * drivebaseSubsystem,
         * translationXSupplier,
         * translationYSupplier,
         * DriverStation.getAlliance().get().equals(Alliance.Red) ? 90 : -90)
         * .alongWith(new PivotAngleCommand(pivotSubsystem, 138)) // FIXME idk
         * .alongWith(new ShooterRampUpCommand(shooterSubsystem,
         * ShooterMode.RAMP_AMP_FRONT)));
         */

        DoubleSupplier rotation = exponential(
                () -> ControllerUtil.deadband(
                        (anthony.getRightTriggerAxis() + -anthony.getLeftTriggerAxis()), .1),
                2);

    DoubleSupplier rotationVelocity =
        () -> -rotation.getAsDouble() * Drive.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * 0.8;
    
    DoubleSupplier rotationAbsolute = () -> anthony.getRightTriggerAxis() - anthony.getLeftTriggerAxis();

    new Trigger(() -> Math.abs(rotationAbsolute.getAsDouble()) > 0.1)
        .whileTrue(
            new RotateVelocityDriveCommand(
                drivebaseSubsystem,
                translationXSupplier,
                translationYSupplier,
                rotationAbsolute,
                anthony.rightBumper()));

        new Trigger(
                () -> Util.vectorMagnitude(anthony.getRightY(), anthony.getRightX()) > Drive.ROTATE_VECTOR_MAGNITUDE)
                .onTrue(
                        new RotateVectorDriveCommand(
                                drivebaseSubsystem,
                                translationXSupplier,
                                translationYSupplier,
                                anthony::getRightY,
                                anthony::getRightX,
                                anthony.rightBumper()));
    }

    /**
     * Adds all autonomous routines to the autoSelector, and places the autoSelector
     * on Shuffleboard.
     */
    private void setupAutonomousCommands() {
        driverView.addString("NOTES", () -> "...win?\nor not.").withSize(4, 1).withPosition(7, 2);

        driverView.add("auto selector", autoSelector).withSize(4, 1).withPosition(7, 0);

        autoDelay = driverView
                .add("auto delay", 0)
                .withWidget(BuiltInWidgets.kNumberSlider)
                .withProperties(Map.of("min", 0, "max", 15, "block increment", .1))
                .withSize(4, 1)
                .withPosition(7, 1)
                .getEntry();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        double delay = autoDelay.getDouble(0);
        return delay == 0
                ? autoSelector.getSelected()
                : new WaitCommand(delay).andThen(autoSelector.getSelected());
    }

  /**
   * @return the scaler to be multipied to the x and y axises
   */
  private static double axisScaler(double xValue, double yValue) {
    double radius = Math.hypot(xValue, yValue);
    return radius < Drive.DEADBAND ? 0 : Math.pow(((radius-Drive.DEADBAND)/(radius*(1-Drive.DEADBAND))),0.75)*(0.95)+0.05;
  }

    private static DoubleSupplier exponential(DoubleSupplier supplier, double exponential) {
        return () -> {
            double val = supplier.getAsDouble();
            return Math.copySign(Math.pow(val, exponential), val);
        };
    }
}
