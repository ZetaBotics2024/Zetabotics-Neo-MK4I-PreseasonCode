// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.SwerveDrive;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.SwerveModuleConstants;
import frc.robot.utils.SwerveModuleAngleOptimizer;

public class SwerveModule{

  private final CANSparkMax m_driveMotor;
  private final CANSparkMax m_turningMotor;

  private SparkMaxPIDController drivingPIDController;
  private SparkMaxPIDController turningPIDController;

  private RelativeEncoder driveRelativeEncoder;

  private CANCoder turningAbsoluteEncoder;
  private double turningAbsoluteEncoderOffset;
  private RelativeEncoder turningRelativeEncoder;

  public SwerveModule(int driveMotorId, int turningMotorId, int turningAbsoluteEncoderId, double turningEncoderOffset, boolean driveMotorRev,
   boolean turnMotorRev) {

    this.m_driveMotor = new CANSparkMax(driveMotorId, MotorType.kBrushless);
    this.m_turningMotor = new CANSparkMax(turningMotorId, MotorType.kBrushless);

    this.driveRelativeEncoder = this.m_driveMotor.getEncoder();

    this.turningAbsoluteEncoder = new CANCoder(turningAbsoluteEncoderId);
    this.turningRelativeEncoder = m_turningMotor.getEncoder();

    this.drivingPIDController = this.m_driveMotor.getPIDController();
    this.turningPIDController = this.m_turningMotor.getPIDController();

    this.m_driveMotor.restoreFactoryDefaults();
    this.m_turningMotor.restoreFactoryDefaults();

    this.m_driveMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 20);
    this.m_turningMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 20);

    this.m_driveMotor.setIdleMode(IdleMode.kBrake);
    this.m_turningMotor.setIdleMode(IdleMode.kBrake);

    this.m_driveMotor.setInverted(driveMotorRev);
    this.m_turningMotor.setInverted(turnMotorRev);

    //this.turningRelativeEncoder.setPositionConversionFactor(SwerveModuleConstants.kRelativeTurningEncoderCPRToDegrees);

    // This may need to be changed
    this.m_driveMotor.setSmartCurrentLimit(20);
    this.m_turningMotor.setSmartCurrentLimit(20);

    this.drivingPIDController.setP(SwerveModuleConstants.kPModuleDriveController);
    this.drivingPIDController.setI(SwerveModuleConstants.kIModuleDriveController);
    this.drivingPIDController.setD(SwerveModuleConstants.kDModuleDriveController);
    this.drivingPIDController.setFF(SwerveModuleConstants.kFModuleDriveController);
    this.drivingPIDController.setIZone(SwerveModuleConstants.kIZoneModuleDriveController);
    this.drivingPIDController.setOutputRange(-1, 1);

    this.turningPIDController.setP(SwerveModuleConstants.kPModuleTurningController);
    this.turningPIDController.setI(SwerveModuleConstants.kIModuleTurningController);
    this.turningPIDController.setD(SwerveModuleConstants.kDModuleTurningController);
    this.turningPIDController.setFF(SwerveModuleConstants.kFModuleTurningController);
    this.turningPIDController.setIZone(SwerveModuleConstants.kIZoneModuleTurningController);
    this.turningPIDController.setOutputRange(-1, 1);

    this.m_driveMotor.burnFlash();
    this.m_turningMotor.burnFlash();

    CANCoderConfiguration turninAbsolutegEncoderConfig = new CANCoderConfiguration();
    this.turningAbsoluteEncoder.configFactoryDefault(); 
    turninAbsolutegEncoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    turninAbsolutegEncoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    turninAbsolutegEncoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    this.turningAbsoluteEncoder.configAllSettings(turninAbsolutegEncoderConfig);
    this.turningAbsoluteEncoderOffset = turningEncoderOffset;

    resetTurningMotorToAbsolute();
  }
    
  private void resetTurningMotorToAbsolute() {
    this.turningRelativeEncoder.setPosition(((this.turningAbsoluteEncoder.getAbsolutePosition() - this.turningAbsoluteEncoderOffset) / 360)* SwerveModuleConstants.kTurningGearRatio);
  }

  private double getTurningEncoderAngleRadiens() {
    return Math.toRadians(getTurningEncoderAngleDegrees().getDegrees());
  }

  private double degreesToCPR(double degrees) {
    return (degrees / 360) * SwerveModuleConstants.kNeoEncoderCPR;
  }

  public Rotation2d getTurningEncoderAngleDegrees() {
    return Rotation2d.fromDegrees(((this.turningRelativeEncoder.getPosition() * SwerveModuleConstants.kNeoEncoderCPR) / SwerveModuleConstants.kRelativeTurningEncoderDegreesToCPRMult) / SwerveModuleConstants.kTurningGearRatio);
  };

  private void setHeadingInDegrees(Rotation2d optimizedDesiredRotation){
    double desiredDegrees = optimizedDesiredRotation.getDegrees(); 
    double desiredEncoderPulses = desiredDegrees * SwerveModuleConstants.kRelativeTurningEncoderDegreesToCPRMult * SwerveModuleConstants.kTurningGearRatio;
    turningPIDController.setReference(desiredEncoderPulses, ControlType.kPosition);
  }

  public double getDriveMotorSpeedInMetersPerSecond() {
    return this.driveRelativeEncoder.getVelocity() * SwerveModuleConstants.kNeoEncoderCPRToMetersPerSecond;
  }

  private double metersPerSecondToCPR(double metersPerSecond) {
    return metersPerSecond / SwerveModuleConstants.kNeoEncoderCPRToMetersPerSecond;
  }

  public double getDistance() {
    return (m_driveMotor.getEncoder().getPosition() * SwerveModuleConstants.kNeoEncoderCPR) * SwerveModuleConstants.kDriveEncoderDistancePerPulse;
  }

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDistance(), new Rotation2d(getTurningEncoderAngleRadiens()));
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveMotorSpeedInMetersPerSecond(), getTurningEncoderAngleDegrees());
  }

  public void setDesiredState(SwerveModuleState desiredState) {
      SwerveModuleState state = SwerveModuleAngleOptimizer.optimize(desiredState, getTurningEncoderAngleDegrees());

      double desiredVelocity = metersPerSecondToCPR(state.speedMetersPerSecond);
      m_driveMotor.getPIDController().setReference(desiredVelocity, CANSparkMax.ControlType.kVelocity);
      this.setHeadingInDegrees(state.angle);
  }
  
}
 

 
 