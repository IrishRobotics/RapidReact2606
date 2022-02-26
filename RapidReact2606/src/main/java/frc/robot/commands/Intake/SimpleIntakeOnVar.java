package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.MOTOR_STATUS;

public class SimpleIntakeOnVar extends CommandBase {
    private IntakeSubsystem m_subsystem;
    private GenericHID controller;

    public SimpleIntakeOnVar(IntakeSubsystem subsystem, GenericHID driveController) {
        m_subsystem = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
        controller=driveController;
      }
    
      // Called when the command is initially scheduled.
      @Override
      public void initialize() {
        m_subsystem.setMode(MOTOR_STATUS.ON);
        m_subsystem.updateMotors(controller.getRawAxis(3));
      }
    
      // Called every time the scheduler runs while the command is scheduled.
      @Override
      public void execute() {
        m_subsystem.updateMotors(controller.getRawAxis(3));
      }
    
      // Returns true when the command should end.
      @Override
      public boolean isFinished() {
        return false;
      }    
}
