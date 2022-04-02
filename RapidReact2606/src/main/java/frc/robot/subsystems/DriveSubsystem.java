// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import edu.wpi.first.wpilibj.drive.DifferentialDrive;
// import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
// import edu.wpi.first.wpilibj.motorcontrol.Talon;
// import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.simulation.AnalogGyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// import com.ctre.phoenix.motorcontrol.can.TalonSRX;
// import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants;
import frc.robot.Robot;

public class DriveSubsystem extends SubsystemBase {
    
    //Left side Motors
    // VictorSP leftMotorFront = new VictorSP(DriveConstants.leftMotorPort1);
    WPI_TalonSRX leftMotorFront = new WPI_TalonSRX(DriveConstants.leftMotorCan1);
    WPI_VictorSPX leftMotorBack  = new WPI_VictorSPX(DriveConstants.leftMotorCan2);
    
    //Right Side Motors
    WPI_TalonSRX rightMotorFront = new WPI_TalonSRX(DriveConstants.rightMotorCan1);
    WPI_VictorSPX rightMotorBack  = new WPI_VictorSPX(DriveConstants.rightMotorCan2);
    // private final MotorControllerGroup rightMotors = new MotorControllerGroup(rightMotorFront, rightMotorBack);

    public void init(){
        leftMotorBack.follow(leftMotorFront);
        rightMotorBack.follow(rightMotorFront);
    }
    //Robots Drive  
    private final DifferentialDrive drive = new DifferentialDrive(leftMotorFront, rightMotorFront);

    //Side Encoders
    private final Encoder leftEncoder = new Encoder(
        DriveConstants.leftEncoderPorts[0], 
        DriveConstants.leftEncoderPorts[1], 
        DriveConstants.isLeftEncoderReversed
    );
    private final Encoder rightEncoder = new Encoder(
            DriveConstants.rightEncoderPorts[0], 
            DriveConstants.rightEncoderPorts[1], 
            DriveConstants.isRightEncoderReversed
    );

    //PID controller creation for each side
    private final PIDController leftPIDController = new PIDController(
        DriveConstants.PIDConsts.Motor.kP, 
        DriveConstants.PIDConsts.Motor.kI, 
        DriveConstants.PIDConsts.Motor.kD
    );
    private final PIDController rightPIDController = new PIDController(
        DriveConstants.PIDConsts.Motor.kP, 
        DriveConstants.PIDConsts.Motor.kI, 
        DriveConstants.PIDConsts.Motor.kD
    );

    //Gyroscope Sensor
    private final AnalogGyro gyro = new AnalogGyro(0);

    //Define Drive Mode
    MODE mode = MODE.ARCADE;

    public enum MODE {
        ARCADE, PERSPECTIVE, TANK
    }
    
    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(DriveConstants.trackWidth);
    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());

    //estimator that corrects for errors in models, encoding, and vision
    DifferentialDrivePoseEstimator estimator = new DifferentialDrivePoseEstimator(
        new Rotation2d(),
        new Pose2d(),
        new MatBuilder<>(Nat.N5(),Nat.N1()).fill(0.02, 0.02, 0.01, 0.02, 0.02), //state standard deviations, how much we trust the estimator model
                                                                                //In form [x, y, theta, dist_l, dist_r]ᵀ
        new MatBuilder<>(Nat.N3(),Nat.N1()).fill(0.02,0.02,0.01), //standard deviation of encoder measurements, how much we trust each encoder and the gyro
                                                                  //In form [dist_l, dist_r, theta]ᵀ
        new MatBuilder<>(Nat.N3(),Nat.N1()).fill(0.01,0.1,0.01) //standard deviation of the computer vision, Trust in our distance and angle measure from the computer vision
                                                                //In form [x, y, theta]ᵀ
    );

    // motor feed calulator
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(DriveConstants.mgStatic, DriveConstants.mgVelocity); // 

    //Vision
    private PhotonCamera cam = new PhotonCamera(Constants.Vision.kCamName);

    // simulation classes
    private final AnalogGyroSim gyroSim = new AnalogGyroSim(gyro);
    private final EncoderSim leftEncoderSim = new EncoderSim(leftEncoder);
    private final EncoderSim rightEncoderSim = new EncoderSim(rightEncoder);
    private final Field2d fieldSim = new Field2d();
    // private final LinearSystem<N2,N2,N2> driveLinearSystem = LinearSystemId.identifyDrivetrainSystem(1.98, 0.2, 1.5, 0.3); // if this wasn't for the sim i would have used constants 
    private final DifferentialDrivetrainSim driveSim = new DifferentialDrivetrainSim(
        // driveLinearSystem, // linear system
        DCMotor.getCIM(2), // number and type of motor to sim
        7.29, // gearing ratio base 7.29:1 - The gearing ratio between motor and whee
        7.5, // jKgMetersSquared - The moment of inertia of the drivetrain about its center.
        60.0, // sim mass
        DriveConstants.wheelRadius, // sim wheel radium
        DriveConstants.trackWidth, // sim track width
        new MatBuilder<>(Nat.N7(), Nat.N1()).fill(0.01, 0.01, 0.0001, 0.05, 0.05, 0.005, 0.005) // stDev of x,x,heading,Vleft,Vright,Xleft,Xright I used the base ones reconmeneded on the api page
    );
    private double lastLeftSpeed  = 0.0;
    private double lastRightSpeed = 0.0;
    SimVisionSystem simVision;


    //PID Controller for Drive
    PIDController PIDControllerDrive = new PIDController(
        DriveConstants.PIDConsts.Drive.kP, 
        DriveConstants.PIDConsts.Drive.kI, 
        DriveConstants.PIDConsts.Drive.kD
    );

    
    /** Creates a new DriveSubsystem. */
    public DriveSubsystem() {
        leftEncoder.setDistancePerPulse(2*Math.PI * (DriveConstants.wheelRadius/DriveConstants.encoderResolution));
        rightEncoder.setDistancePerPulse(2*Math.PI * (DriveConstants.wheelRadius/DriveConstants.encoderResolution));

        resetEncoders();

        rightMotorFront.setInverted(true);

        SmartDashboard.putData("Field", fieldSim);
    }

    /**
     * @param fwd, forward movement
     * @param rot, rotational movemnent
     */
    public void drive(double fwd, double rot){
        switch(mode){
            case ARCADE:
                setSpeeds(kinematics.toWheelSpeeds(new ChassisSpeeds(fwd,0,rot)));
            break;
            case PERSPECTIVE:
                double[] params = getDriveParameters(rot, -fwd);
                setSpeeds(kinematics.toWheelSpeeds(new ChassisSpeeds(params[0], 0, params[1])));
            break;
            default:
            break;
        }
    }

    // gets drive wheel speeds of each side of the robot drive train from the encoders
    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(leftEncoder.getRate(), rightEncoder.getRate());
    }

    

    /**
   * Gets the average distance of the two encoders.
   *
   * @return the average of the two encoder readings
   */
    public double getAverageEncoderDistance() {
        return (leftEncoder.getDistance() + rightEncoder.getDistance()) / 2.0;
    }

    

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        updateOdometry();
        fieldSim.setRobotPose(odometry.getPoseMeters());
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
        // To update our simulation, we set motor voltage inputs, update the
        // simulation, and write the simulated positions and velocities to our
        // simulated encoder and gyro. We negate the right side so that positive
        // voltages make the right side move forward.
        driveSim.setInputs(lastLeftSpeed, lastRightSpeed);
        driveSim.update(0.02);

        leftEncoderSim.setDistance(driveSim.getLeftPositionMeters());
        leftEncoderSim.setRate(driveSim.getLeftVelocityMetersPerSecond());
        rightEncoderSim.setDistance(driveSim.getRightPositionMeters());
        rightEncoderSim.setRate(driveSim.getRightVelocityMetersPerSecond());
        gyroSim.setAngle(-driveSim.getHeading().getDegrees());

        if (null == simVision) {
            double camDiagFOV = 75.0; // degrees
            double camPitch = 15.0; // degrees
            double camHeightOffGround = 0.85; // meters
            double maxLEDRange = 20; // meters
            int camResolutionWidth = 640; // pixels
            int camResolutionHeight = 480; // pixels
            double minTargetArea = 10; // square pixels
            simVision = new SimVisionSystem(VisionConstants.camName, camDiagFOV, camPitch, VisionConstants.cameraToRobot,
            camHeightOffGround, maxLEDRange, camResolutionWidth, camResolutionHeight, minTargetArea);
            simVision.addSimVisionTarget(VisionConstants.farTarget);
        }
        simVision.processFrame(driveSim.getPose());
    }


    /**
     * ******************
     *  ASSORTED GETTERS
     * ******************
     */


    /** @return the kinematics encoder*/
    public DifferentialDriveKinematics kine() {
        return kinematics;
    }

    /** @return the left drive encoder*/
    public Encoder getLeftEncoder() {
        return leftEncoder;
    }

    /** @return the right drive encoder*/
    public Encoder getRightEncoder() {
        return rightEncoder;
    }

    /**@return robots heading in degress  */
    public double getHeading(){
        return Math.IEEEremainder(gyro.getAngle(), 360) * (DriveConstants.isGyroReversed ? -1.0 : 1.0);
    }

    /**@return returns the drive mode*/
    public MODE getMode() {
        return mode;
    }

    /**@return The turn rate of the robot, in degrees per second */
    public double getTurnRate(){
        return gyro.getRate() * (DriveConstants.isGyroReversed ? -1.0 : 1.0);
    }

    /**@return the current robot position. */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    protected double[] getDriveParameters(double x, double y) {
        double[] retVal = { 0, 0 };
    
        PIDControllerDrive.enableContinuousInput(-180.0, 180.0);
        PIDControllerDrive.setTolerance(0);
        // Drive with arcade drive.
        // That means that the Y axis drives forward
        // and backward, and the X turns left and right.
        double gyro_angle = gyro.getAngle();
        double rad = Math.atan2(y, -x);
        double deg = MathUtil.inputModulus(rad * (180 / Math.PI), -180, 180);
    
        double power = Math.sqrt(x * x + y * y);
    
        if (power > 1) {
          power = 1;
        }
    
        deg = MathUtil.inputModulus(deg, -180, 180);
        double positionError = MathUtil.inputModulus(deg - gyro_angle, -180, 180);
        double value = PIDControllerDrive.calculate(gyro_angle, deg);
        double rot = 0.0;
        if (Math.abs(positionError) > 0) {
          rot = MathUtil.clamp(value / Math.abs(positionError), -1.0, 1.0);
        }
    
        if (PIDControllerDrive.atSetpoint()) {
          rot = 0;
        }
    
        if (Math.abs(rot) > .75) {
          power = 0;
        }
    
        retVal[0] = power;
        retVal[1] = -rot;
    
        return retVal;
    }
    /**@return returns photon cam */
    public PhotonCamera getCamera() {
        return cam;
    }


    /**
     * ******************
     *  ASSORTED Setters
     * ******************
     */

    /**@param maxOutput the max output drive will be constrained to */
    public void setMaxOutput(double maxOutput){
        drive.setMaxOutput(maxOutput);
    }
    /**
     * 
     * @param m mode
     */
    public void setMode(MODE m){
        mode = m;
    }

    /**@param speeds wheel speed */
    public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
        var leftFeedforward = feedforward.calculate(speeds.leftMetersPerSecond);
        var rightFeedforward = feedforward.calculate(speeds.rightMetersPerSecond);
        double leftOutput = leftPIDController.calculate(leftEncoder.getRate(), speeds.leftMetersPerSecond);
        double rightOutput = rightPIDController.calculate(rightEncoder.getRate(), speeds.rightMetersPerSecond);
    
        leftMotorFront.setVoltage(leftOutput + leftFeedforward);
        rightMotorFront.setVoltage(rightOutput + rightFeedforward);
    
        if (Robot.isSimulation()) {
          lastLeftSpeed = leftOutput + leftFeedforward;
          lastRightSpeed = rightOutput + rightFeedforward;
        }
    }
    /**
     * 
     * @param leftVolts sets voltage of left motor group
     * @param rightVolts sets voltage of right motor group
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotorFront.setVoltage(leftVolts);
        rightMotorFront.setVoltage(-rightVolts);
    
        if (Robot.isSimulation()) {
          lastLeftSpeed = leftVolts;
          lastRightSpeed = rightVolts;
        }
    
        drive.feed();
    }
    
    
    /**
     * ******************
     *  ASSORTED Resets
     * ******************
     */
    
     /** Resets the drive encoders to currently read a position of 0. */
    public void resetEncoders() {
        leftEncoder.reset();
        rightEncoder.reset();
    }
    
    /** Zeroes the heading of the robot. */
    public void zeroHeading() {
        gyro.reset();
    }
    
    /** Resets robot odometry. */
    public void resetOdometry(Pose2d pose) {
        leftEncoder.reset();
        rightEncoder.reset();
        driveSim.setPose(pose);
        odometry.resetPosition(pose, gyro.getRotation2d());
    }

     /** Update robot odometry. */
    public void updateOdometry() {
        odometry.update(gyro.getRotation2d(), leftEncoder.getDistance(), rightEncoder.getDistance());
        estimator.update(gyro.getRotation2d(), getWheelSpeeds(), leftEncoder.getDistance(),
            leftEncoder.getDistance());

        PhotonPipelineResult res = cam.getLatestResult();

        if (res.hasTargets()) {
        double imageCaptureTime = Timer.getFPGATimestamp() - res.getLatencyMillis();
        Transform2d camToTargetTrans = res.getBestTarget().getCameraToTarget();
        Pose2d camPose = VisionConstants.farTargetPose.transformBy(camToTargetTrans.inverse());
        estimator.addVisionMeasurement(camPose.transformBy(VisionConstants.cameraToRobot), imageCaptureTime);
        }

    }
    public Object stop() {
        return null;
      }
}