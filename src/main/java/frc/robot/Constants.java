// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;


public final class Constants {

    // Swerve constants
    public static final class SwerveConstants {

        // Rotor IDs
        public static final int kLeftFrontRotorID = 1;
        public static final int kRightFrontRotorID = 4;
        public static final int kLeftRearRotorID = 8;
        public static final int kRightRearRotorID = 6;

        // Throttle IDs
        public static final int kLeftFrontThrottleID = 2;
        public static final int kRightFrontThrottleID = 3;
        public static final int kLeftRearThrottleID = 7;
        public static final int kRightRearThrottleID = 5;

        // Rotor encoder IDs
        public static final int kLeftFrontCANCoderID = 9;
        public static final int kRightFrontCANCoderID = 11;
        public static final int kLeftRearCANCoderID = 12;
        public static final int kRightRearCANCoderID = 10;

        // Rotor encoder & motor inversion
        public static final boolean kRotorEncoderDirection = false;
        // drive encoder
        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kRearLeftDriveEncoderReversed = false;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kRearRightDriveEncoderReversed = false;


        public static final boolean kRotorMotorInversion = true;

        // IMU ID
        public static final int kImuID = 0;

        // Rotor encoder offsets
        // Rotor encoder 偏移量
        public static final double kLeftFrontRotorOffset =-137.988;
        public static final double kRightFrontRotorOffset =162.773-71.613+180;
        public static final double kLeftRearRotorOffset = 112.764;
        public static final double kRightRearRotorOffset = 112.5 ;

        // Swerve kinematics (order: left front, right front, left rear, right rear)
        // Swerve kinematics（順序：左前，右前，左後，右後）
        public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
            new Translation2d(0.3, 0.3), 
            new Translation2d(0.3, -0.3), 
            new Translation2d(-0.3, 0.3),
            new Translation2d(-0.3, -0.3)
        );

        // Rotor PID constants
       // public static final double kRotor_kF = 0.1;
        public static final double kRotor_kP = 0.01;  // <--------------------------
        public static final double kRotor_kI = 0.0;
        public static final double kRotor_kD = 0.0;

        // Velocity & acceleration of swerve
        // Swerve 最大速度 / 加速度
        public static final double kMaxVelocityMetersPerSecond = 13.0;
        public static final double kMaxAccelerationMetersPerSecond = 13.0;

        // Wheel diameter
        // 輪徑
        public static final double kWheelDiameterMeters = 0.1; // wheel diameter <---------------------------------------
        
        // Throttle gear ratio
        // (number of turns it takes the motor to rotate the rotor one revolution)
        // Throttle 齒輪比率
        // （馬達轉動輪子一圈所需的圈數）
        public static final double kThrottleGearRatio = 150/7; // <-----------------------------------------------

        // Throttle velocity conversion constant
        // Throttle 速度轉換 Constant
        // This value will be multiplied to the raw RPM of the throttle motor
        // and should convert it to meters per second
        // This is the general formula: 
        //     (1.0 / GEAR RATIO / 60.0_seconds) * WHEEL DIAMETER * Math.PI;
        public static final double kThrottleVelocityConversionFactor = 
            (1/kThrottleGearRatio/60)*kWheelDiameterMeters*Math.PI;
        
        // Pathing PID constants 
        public static final double kPathingX_kP = 0.1;
        public static final double kPathingX_kI = 0.0;
        public static final double kPathingX_kD = 0.0;

        public static final double kPathingY_kP = 0.1;
        public static final double kPathingY_kI = 0.0;
        public static final double kPathingY_kD = 0.0;

        public static final double kPathingTheta_kP = 0.1;
        public static final double kPathingTheta_kI = 0.0;
        public static final double kPathingTheta_kD = 0.0;
        public static double kLkFrontLeftDriveEncoderReversed;
    }
    // Voltage compensation
    public static final double kVoltageCompensation = 9.0;
    
    // Controller port
    public static final int kControllerPort = 0;
}
