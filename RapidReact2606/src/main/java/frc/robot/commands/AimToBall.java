package frc.robot.commands;

import java.util.Iterator;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.node.ArrayNode;

import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.MODE;

public class AimToBall extends CommandBase {
    private DriveSubsystem m_subsystem;
    private boolean m_finished = false;
    
    final double LINEAR_P = 0.1;

    final double LINEAR_D = 0.0;

    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    final double ANGULAR_P = 0.1;

    final double ANGULAR_D = 0.0;

    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
    private NetworkTable table;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AimToBall(DriveSubsystem subsystem) {
        m_subsystem = subsystem;
        m_finished = true;
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        table = inst.getTable("ML");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_finished = false;
        NetworkTableEntry xEntry = table.getEntry("best_blue_x");
        NetworkTableEntry widthEntry = table.getEntry("width"); 
        if (xEntry.getNumber(Integer.valueOf(0)) != Integer.valueOf(0)) { 
            Integer width = (Integer) widthEntry.getNumber(Integer.valueOf(0));
            Integer best_target_x = (Integer) xEntry.getNumber(0);
            int error = best_target_x - width/2;
            double turnSpeed = turnController.calculate(error, 0);

            m_subsystem.drive(0.0, turnSpeed);

        }
    }
}