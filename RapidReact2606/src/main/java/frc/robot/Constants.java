// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.photonvision.SimVisionTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;https://github.com/IrishRobotics/RapidReact2606/pull/3/conflict?name=RapidReact2606%252Fsrc%252Fmain%252Fjava%252Ffrc%252Frobot%252FConstants.java&ancestor_oid=329cd5adbacf3549e8f52c0b1cf4f6e02348e585&base_oid=da71f73212f6210932ed1d4dcc45103942df6603&head_oid=323c91226fb3c7896c741bbbca4db8ea925cb2e4
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
        public static final int leftMotorCan1  = 0;
        public static final int leftMotorCan2  = 1;
        public static final int rightMotorCan1 = 2;
        public static final int rightMotorCan2 = 3;

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
        public static final int firstDIO = 0;
        public static final int middleDIO = 1;
        public static final int lastDIO = 2;
        public static final int motorControllerPort = 23;
    }

    public static final class ShooterConstants{
        public static final int shooterCan = 9;
        public static final double speed = 1.0;
    }
}
