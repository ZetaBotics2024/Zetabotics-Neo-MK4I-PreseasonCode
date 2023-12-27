// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDeadband = .1;
  }

  public static class SwerveDriveConstants {
    
    public static final int kFrontLeftDriveMotorId = 1;
    public static final int kFrontRightDriveMotorId = 2;
    public static final int kBackLeftDriveMotorId = 3;
    public static final int kBackRightDriveMotorId = 4;


    public static final int kFrontLeftTurnMotorId = 5;
    public static final int kFrontRightTurnMotorId = 6;
    public static final int kBackLeftTurnMotorId = 7;
    public static final int kBackRightTurnMotorId = 8;


    public static final int kFrontLeftTurnEncoderId = 9;
    public static final int kFrontRightTurnEncoderId = 10;
    public static final int kBackLeftTurnEncoderId = 11;
    public static final int kBackRightTurnEncoderId = 12;

    // Make Sure to set these
    public static final double kFrontLeftTurnEncoderOffset = 274.746;
    public static final double kFrontRightTurnEncoderOffset = 293.115;
    public static final double kBackLeftTurnEncoderOffset = 70.137;
    public static final double kBackRightTurnEncoderOffset = 120.234;

    public static final boolean kFrontLeftDriveMotorRev =  false;
    public static final boolean kFrontRightDriveMotorRev =  false;
    public static final boolean kBackLeftDriveMotorRev = false;
    public static final boolean kBackRightDriveMotorRev =  false;

    public static final boolean kFrontLeftTurnMotorRev =  true;
    public static final boolean kFrontRightTurnMotorRev =  true;
    public static final boolean kBackLeftTurnMotorRev = true;
    public static final boolean kBackRightTurnMotorRev =  true;

    public static final int kGyroId = 13;
    public static final boolean kGyroReversed = false;

    public static final double kMaxSpeedMetersPerSecond = 4.4196;
    public static final double kMaxRotationAnglePerSecond = .6;

    public static final double kRadiusFromCenterToSwerves = 1;

    // Last years values
    public static final double kRadiusFromCenterToSwerveDrives = 0;
    public static final double kDistanceBetweenCentersOfRightAndLeftWheels = 0.6096;
    public static final double kDistanceBetweenCentersOfFrontAndBackWheels = 0.6096;

    public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(kDistanceBetweenCentersOfFrontAndBackWheels / 2, kDistanceBetweenCentersOfRightAndLeftWheels / 2),
                new Translation2d(kDistanceBetweenCentersOfFrontAndBackWheels / 2, -kDistanceBetweenCentersOfRightAndLeftWheels / 2),
                new Translation2d(-kDistanceBetweenCentersOfFrontAndBackWheels / 2, kDistanceBetweenCentersOfRightAndLeftWheels / 2),
                new Translation2d(-kDistanceBetweenCentersOfFrontAndBackWheels / 2, -kDistanceBetweenCentersOfRightAndLeftWheels / 2));

    public static final double kPModuleTurningController = .2;
    public static final double kIModuleTurningController = 0.0001;
    public static final double kDModuleTurningController = .01;

    public static final double kPostitionToleranceDegrees = .1;
    public static final double kVelocityToleranceDegreesPerSec = 2.0;

    public static final double kMaxModuleAngularSpeedDegreesPerSecond =  30;
    public static final double kMaxModuleAngularAccelDegreesPerSecondSquared = 30;
  }

  public static class SwerveModuleConstants {
    public static final double kMaxModuleAngularSpeedRadiansPerSecond = 4 * 2 * Math.PI;
    public static final double kMaxModuleAngularAccelerationRadiansPerSecondSquared = 4 * 2 * Math.PI;

    // Set to the last years values
    public static final double kPModuleDriveController = 0.0006000000142492354;
    public static final double kIModuleDriveController = 0;
    public static final double kDModuleDriveController = 0.0;
    public static final double kFModuleDriveController = 0.0;
    public static final double kIZoneModuleDriveController = 0.0;

    public static final double kPModuleTurningController = 17;
    public static final double kIModuleTurningController = 0;
    public static final double kDModuleTurningController = 0;
    public static final double kFModuleTurningController = 0.0;
    public static final double kIZoneModuleTurningController = 0.25;
  
    // Updated for this year
    public static final double kAbsoluteTurningEncoderCPR = 4096;
    public static final double kNeoEncoderCPR = 4096;
    public static final double kMaxRPM = 5676;
    public static final double kWheelDiameterMeters = 0.1016;
    public static final double kDriveGearRatio = 6.75/1;
    public static final double kTurningGearRatio = 150/7; 

    public static final double kAbsoluteTurningEncoderCPRToDegrees = 
    (kAbsoluteTurningEncoderCPR / kAbsoluteTurningEncoderCPR) * 360;

    public static final double kAbsoluteTurningEncoderCPRToDegreesMult = 360 / 4096;

    public static final double kRelativeTurningEncoderDegreesToCPRMult = kNeoEncoderCPR / 360;
    //((kNeoEncoderCPR / kNeoEncoderCPR) * 360) * kTurningGearRatio;

    public static final double kWheelDistancePerRotation = kWheelDiameterMeters * Math.PI;

    public static final double kNeoEncoderCPRToMetersPerSecond = 10 / kNeoEncoderCPR / kDriveGearRatio  * kWheelDistancePerRotation;

    public static final double kDriveEncoderDistancePerPulse =
        ((kWheelDiameterMeters * Math.PI) / kDriveGearRatio) / kNeoEncoderCPR;

    public static final double kTurningEncoderRadiansPerPulse =
        // Assumes the encoders are on a 1:1 reduction with the module shaft.
        (2 * Math.PI) / kAbsoluteTurningEncoderCPR;
  }

  public static final class FieldConstants {
    public static final double kLength = Units.feetToMeters(54);
    public static final double kWidth = Units.feetToMeters(27);
  }

  public static final class VisionConstants {
    //15.25 inch h
    //19 3/4 v
    public static final String cameraName = "photonvision";
    public static final Transform3d robotToCam = new Transform3d(new Translation3d(.26, .2, 0), new Rotation3d()); 
    //public static final Transform3d ROBOT_TO_CAMERA = robotToCam.inverse();
    
  };

  public static final class AutoConstants {
    public static HashMap<String, Command> namedEventMap = new HashMap<>();
    public static Alliance alliance;
    public static final double kMaxAutonSpeedInMetersPerSecond = 4.1;
    public static final double kMaxAutonAccelerationInMetersPerSecondSqr = 4.1;

    // These need to be changed to have something to do with pi
    public static final double kMaxAutonAngulerSpeedInMetersPerSecond = 4.1;
    public static final double kMaxAutonAngulerAccelerationInMetersPerSecondSqr = 4.1;
  }
}
