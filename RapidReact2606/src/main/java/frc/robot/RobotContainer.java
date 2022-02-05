// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import frc.robot.Constants.IOConstants;
import frc.robot.commands.SetIndexerMode;
import frc.robot.commands.SetShooterMode;
// import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IndexerSubsystem.MODE;
// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
// import edu.wpi.first.wpilibj.GenericHID;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem robotDrive = new DriveSubsystem();
  private GenericHID driveController = IOConstants.isXbox ? new XboxController(IOConstants.DriverControllerPort) : new Joystick(IOConstants.DriverControllerPort); 
  private IndexerSubsystem indexer = new IndexerSubsystem();
  private ShooterSubsystem shooter = new ShooterSubsystem();  


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //Drive Controller
    // Configure the button bindings
    configureButtonBindings();

    robotDrive.setDefaultCommand(
      new RunCommand(
        () ->
             robotDrive.drive(
              -(Math.abs(driveController.getRawAxis(1)) >0.1? driveController.getRawAxis(1):0.0),
              -(Math.abs(driveController.getRawAxis(0)) >0.1? driveController.getRawAxis(0):0.0)),
               robotDrive));

    indexer.setDefaultCommand(new RunCommand(
      () ->
        indexer.setMode(IndexerSubsystem.MODE.OFF),indexer
      ));
    shooter.setDefaultCommand(new RunCommand(
        () ->
          shooter.setMode(ShooterSubsystem.MODE.OFF),shooter
        ));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    final JoystickButton indexButton = new JoystickButton(driveController, IOConstants.indexButton);
    indexButton.toggleWhenPressed(new SetIndexerMode(indexer, IndexerSubsystem.MODE.INTAKE));
    final JoystickButton shooterButton = new JoystickButton(driveController, IOConstants.shooterButton);
    shooterButton.toggleWhenPressed(new SetShooterMode(shooter, ShooterSubsystem.MODE.SHOOT));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    // return ExampleCommand;
    return null;
  }
}
