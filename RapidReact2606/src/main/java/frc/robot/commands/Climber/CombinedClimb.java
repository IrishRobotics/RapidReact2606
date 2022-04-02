// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.ClimbSubsystem.MOTOR_DIRECTION;
import frc.robot.subsystems.ClimbSubsystem.MOTOR_STATUS;

import java.lang.module.ModuleDescriptor.Requires;

import edu.wpi.first.wpilibj.GenericHID;
// import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class CombinedClimb extends CommandBase {
  private GenericHID driveController;
  private ClimbSubsystem climbSubsystem;
  public void CombinedClimb(GenericHID dc, ClimbSubsystem cs){
    driveController = dc;
    climbSubsystem = cs;
    // Requires
  }
}
