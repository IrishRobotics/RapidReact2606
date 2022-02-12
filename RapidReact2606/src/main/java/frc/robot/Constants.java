// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.SimVisionTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d; 
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static final class DriveConstants{
        //Motor Ports for the Drive System
        public static final int leftMotorCan1  = 7;//front
        public static final int leftMotorCan2  = 5;//back
        public static final int rightMotorCan1 = 3;//front
        public static final int rightMotorCan2 = 8;//back

        //Encoder Ports and Reverse Cases
        public static final int[] leftEncoderPorts = new int[] { 0, 1 };
        public static final int[] rightEncoderPorts = new int[] { 2, 3 };
        public static final boolean isLeftEncoderReversed = false;
        public static final boolean isRightEncoderReversed = true;
        public static final int encoderResolution = -4096;

        // PID Coefficients
        public static final class PIDConsts{
            //Motor PID Coefficients
            public static final class Motor{
                public static final double kP = 8.5;
                public static final double kI = 0;
                public static final double kD = 0;
            }
            
            //Drive PID Coefficients
            public static final class Drive{
                public static final double kP = 1.0;
                public static final double kI = .02;
                public static final double kD = 0.0;
            }

            //Stablization PID Coefficients
            public static final class Stability{
                public static final double kP = 1.0;
                public static final double kI = 0.5;
                public static final double kD = 0.0;
            }
        }

        //Speed Maxes for PID Movement
        public static final double MaxLinearSpeed = 3.0; //in meters
        public static final double MaxAngularSpeed = Math.PI; // in radians

        //Drive Train Measurements
        public static final double trackWidth = 0.762; // width from center of back wheel to center of front wheel in meters
        public static final double wheelRadius = 0.0508; // radius of wheels in meters

        //Motor Gains
        public static final double mgStatic = 1; // in volts
        public static final double mgVelocity = 3; // in voltseconds per meter
        
        //Gyroscope direction
        public static final boolean isGyroReversed = false;
    }

    public static final class IOConstants{
        public static final int DriverControllerPort = 0; 
        public static final boolean isXbox = true;
        public static final int indexButton = 4;
        public static final int shooterButton = 3;
    }
    public static final class VisionConstants{
        //Target Position
        public static final double targetWidth  = Units.inchesToMeters(41.30) - Units.inchesToMeters(6.70); // in meters
        public static final double targetHeight = Units.inchesToMeters(98.19) - Units.inchesToMeters(81.19); // in meters
        public static final double targetHeightAboveGround = Units.inchesToMeters(81.19); // in meters
        
        // Camera Name
        public static final String camName = "video";
        
        //Cordinate Tranformations
        public static final Translation2d translation = new Translation2d(.5, 0); //centers horizontal cords
        public static final Rotation2d rotation = new Rotation2d(0.0); // fixes rotation
        public static final Transform2d cameraToRobot = new Transform2d(translation, rotation); //proforms transformations
        
        //Desired Position
        public static final double farTgtXPos = Units.feetToMeters(54);
        public static final double farTgtYPos = Units.feetToMeters(27 / 2) - Units.inchesToMeters(43.75) - Units.inchesToMeters(48.0 / 2.0);
        public static final Pose2d farTargetPose = new Pose2d(new Translation2d(farTgtXPos, farTgtYPos), new Rotation2d(0.0));
        
        //Sim Position
        public static final org.photonvision.SimVisionTarget farTarget = new SimVisionTarget(farTargetPose,targetHeightAboveGround, targetWidth, targetHeight);
    }
    public static final class IndexerConstants{
        public static final int firstDIO = 11;
        public static final int middleDIO = 12;
        public static final int lastDIO = 13;
        public static final int motorControllerPort = 9;
    }

    public static final class ShooterConstants{
        public static final int shooterCan = 12;
        public static final double shootSpeed = 1.0;
        public static final double coastSpeed = 0.1;
    }
    public static final class IntakeConstants{
        public static final int intakeCan = 1;
    }
}
