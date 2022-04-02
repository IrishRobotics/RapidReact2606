// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.sql.Driver;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.Auto.OneBallAuto;
import frc.robot.commands.Auto.AimToBall;
import frc.robot.commands.Auto.AimToTarget;
import frc.robot.commands.Auto.AutoDriveBack;
import frc.robot.commands.Indexer.SimpleIndexerOn;
import frc.robot.commands.Intake.SimpleIntakeOn;
import frc.robot.commands.Intake.SimpleIntakeOnVar;
import frc.robot.commands.Shooter.SimpleShooterOn;
// import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// import edu.wpi.first.wpilibj.GenericHID;
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
  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private final IntakeSubsystem intakeSystem = new IntakeSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final IndexerSubsystem indexSubsystem = new IndexerSubsystem();

  private GenericHID driveController = IOConstants.isXbox ? new XboxController(IOConstants.DriverControllerPort)
      : new Joystick(IOConstants.DriverControllerPort);
  private JoystickButton a_button = null;
  private JoystickButton b_button = null;
  private JoystickButton x_button = null;
  private JoystickButton y_button = null;
  private JoystickButton left_bumper;
  private JoystickButton right_bumper;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureButtonBindings();

    robotDrive.setDefaultCommand( 
      
      (new RunCommand(
        () -> 
             robotDrive.drive(
              -(Math.abs(driveController.getRawAxis(0)) >0.1? driveController.getRawAxis(0):0.0),
              -(Math.abs(driveController.getRawAxis(1)) >0.1? driveController.getRawAxis(1):0.0)),
               robotDrive))
               
               //.alongWith(new SimpleIntakeOnVar(intakeSystem, (driveController.getRawAxis(3)))
    
    
    );
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
    a_button = new JoystickButton(driveController, XboxController.Button.kA.value);
    b_button = new JoystickButton(driveController, XboxController.Button.kB.value);
    x_button = new JoystickButton(driveController, XboxController.Button.kX.value);
    y_button = new JoystickButton(driveController, XboxController.Button.kY.value);

    left_bumper = new JoystickButton(driveController, XboxController.Button.kLeftBumper.value);
    right_bumper = new JoystickButton(driveController, XboxController.Button.kRightBumper.value);

    a_button.whileHeld(new SimpleIntakeOn(intakeSystem));
    x_button.whenHeld(new SimpleIndexerOn(indexSubsystem));
    y_button.toggleWhenPressed(new SimpleShooterOn(shooterSubsystem));

    left_bumper.whileHeld(new AimToTarget(robotDrive));
    right_bumper.whileHeld(new AimToBall(robotDrive, driveController));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return ExampleCommand;cdcd
    return new OneBallAuto(shooterSubsystem, indexSubsystem);
  }
}
