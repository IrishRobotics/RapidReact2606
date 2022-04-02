package frc.robot.commands.Auto;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.MODE;

public class AimToTarget extends CommandBase{
    private DriveSubsystem m_subsystem;
    private boolean m_finished = false;
    
    final double LINEAR_P = 0.1;
    final double LINEAR_D = 0.1;
    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    final double ANGULAR_P = 0.1;
    final double ANGULAR_D = 0.1;
    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AimToTarget(DriveSubsystem subsystem) {
        m_subsystem = subsystem;
        m_finished = true;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    // Called every time the scheduler runs while the command is scheduled.
     @Override
     public void execute() {
         m_finished = false;
         m_subsystem.setMode(MODE.ARCADE);
 
         PhotonPipelineResult result = m_subsystem.getCamera().getLatestResult();
 
         if (result.hasTargets()) {
 
             // First calculate range
 
             double range =
 
                     PhotonUtils.calculateDistanceToTargetMeters(
 
                             .5,
 
                             Constants.Vision.targetHeight,
 
                             0,
 
                             Units.degreesToRadians(result.getBestTarget().getPitch()));
 
             // Use this range as the measurement we give to the PID controller.
 
             // -1.0 required to ensure positive PID controller effort _increases_ range
 
             double forwardSpeed = forwardController.calculate(range, Constants.Vision.kFarTgtXPos);
 
             // Also calculate angular power
 
             // -1.0 required to ensure positive PID controller effort _increases_ yaw
 
             double turnSpeed = turnController.calculate(result.getBestTarget().getYaw(), 0);
 
             m_subsystem.drive(forwardSpeed, turnSpeed);
 
         } else {
 
             m_finished = true;
         }
 
         // Use our forward/turn speeds to control the drivetrain
 
     }
}
