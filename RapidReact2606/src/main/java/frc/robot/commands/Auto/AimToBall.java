package frc.robot.commands.Auto;

// import java.util.Iterator;

// import com.fasterxml.jackson.core.JsonProcessingException;
// import com.fasterxml.jackson.databind.JsonNode;
// import com.fasterxml.jackson.databind.ObjectMapper;
// import com.fasterxml.jackson.databind.node.ArrayNode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
// import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
// import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.IntakeSubsystem.MOTOR_STATUS;

public class AimToBall extends CommandBase {
    private DriveSubsystem m_subsystem;
    private IntakeSubsystem intakeSubsystem;
    private boolean m_finished = false;

    final double LINEAR_P = 0.6;

    final double LINEAR_D = 0.0;

    PIDController forwardController = new PIDController(LINEAR_P, 0, LINEAR_D);

    final double ANGULAR_P = 0.1;// SmartDashboard.getNumber("Angular Pos", 0.1);

    final double ANGULAR_D = 0.01;

    PIDController turnController = new PIDController(ANGULAR_P, 0, ANGULAR_D);
    private NetworkTable table;

    private boolean isBlue;

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public AimToBall(DriveSubsystem subsystem, IntakeSubsystem intakeSub) {
        m_subsystem = subsystem;
        intakeSubsystem = intakeSub;

        m_finished = true;
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(subsystem);
    }

    public AimToBall(DriveSubsystem robotDrive, GenericHID driveController, IntakeSubsystem intakeSub) {
        m_subsystem = robotDrive;
        intakeSubsystem = intakeSub;
        
        m_finished = true;
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(robotDrive);
    }

    @Override
    public void initialize() {
        System.out.println("Aiming to Ball");
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        table = inst.getTable("ML");
        intakeSubsystem.setMode(MOTOR_STATUS.ON);
        intakeSubsystem.updateMotors();
    }

    @Override
    public boolean isFinished(){
        return m_finished;
    }

    @Override
    public void end(boolean interrupted){
        intakeSubsystem.setMode(MOTOR_STATUS.OFF);
        intakeSubsystem.updateMotors();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //Update intake
        intakeSubsystem.updateMotors();

        //Find and Update Movement
        isBlue = DriverStation.getAlliance() == DriverStation.Alliance.Blue;
        m_finished = false;
        NetworkTableEntry xEntry = table.getEntry(isBlue ? "best_blue_x" : "best_red_x");
        NetworkTableEntry sizeEntry = table.getEntry(isBlue ? "best_blue_size" : "best_red_size");
        System.out.println(xEntry);
        NetworkTableEntry widthEntry = table.getEntry("width");
        double turnSpeed = 0.0;
        double linearSpeed = 0.0;
        if ((Double) xEntry.getNumber(Double.valueOf(0.0)) > Double.valueOf(1.0)) {
            Double width = (Double) widthEntry.getNumber(Double.valueOf(0));
            Double best_target_x = (Double) xEntry.getNumber(0);
            System.out.println(best_target_x);
            double errorAng = best_target_x - width / 2.0;
            m_finished = errorAng < 10;
            if (Math.abs(errorAng) > 5) {
                // System.out.println(errorAng);
                // error = error / 160.0;
                turnSpeed = turnController.calculate(errorAng, 0);
                turnSpeed = turnSpeed * (1.0 / 160.0) * 0.5;
            }
        }
        if ((Double) sizeEntry.getNumber(Double.valueOf(0.0)) > Double.valueOf(1.0)) {
            Double size = (Double) sizeEntry.getNumber(Double.valueOf(0));
            if(Math.abs(size-28000) > 1500){
                double errorLin = size - 28000;
                linearSpeed = forwardController.calculate(errorLin/12000.0, 0.0);
            }
        }
        //m_subsystem.drive(linearSpeed, -turnSpeed);
        SmartDashboard.putNumber("linear Speed", linearSpeed);
        SmartDashboard.putNumber("turnSpeed", turnSpeed);
        m_subsystem.drive(linearSpeed,turnSpeed);
    }
}