package frc.robot.subsystems.SwerveDrive;

import java.util.stream.IntStream;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.REVLibError;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDriveConstants;

public class DriveSubsystem extends SubsystemBase {

    private ChassisSpeeds desiredChassisSpeeds;

    private final SwerveModule frontLeftSwerveModule;
    private final SwerveModule frontRightSwerveModule;
    private final SwerveModule backLeftSwerveModule;
    private final SwerveModule backRightSwerveModule;

    private Pigeon2 m_gyro;
    //private Pose2d startingPosition = new Pose2d(0, 0, new Rotation2d(0));

    private final ProfiledPIDController turningPIDController; 

    public DriveSubsystem() {
        this.frontLeftSwerveModule =  new SwerveModule(
            SwerveDriveConstants.kFrontLeftDriveMotorId, SwerveDriveConstants.kFrontLeftTurnMotorId, SwerveDriveConstants.kFrontLeftTurnEncoderId,
             SwerveDriveConstants.kFrontLeftTurnEncoderOffset, SwerveDriveConstants.kFrontLeftTurnMagnetOffset, SwerveDriveConstants.kFrontLeftDriveMotorRev, SwerveDriveConstants.kFrontLeftTurnMotorRev);
        
        this.frontRightSwerveModule = new SwerveModule(
            SwerveDriveConstants.kFrontRightDriveMotorId, SwerveDriveConstants.kFrontRightTurnMotorId, SwerveDriveConstants.kFrontRightTurnEncoderId,
             SwerveDriveConstants.kFrontRightTurnEncoderOffset, SwerveDriveConstants.kFrontRightTurnMagnetOffset, SwerveDriveConstants.kFrontRightDriveMotorRev, SwerveDriveConstants.kFrontRightTurnMotorRev);

        this.backLeftSwerveModule = new SwerveModule(
            SwerveDriveConstants.kBackLeftDriveMotorId, SwerveDriveConstants.kBackLeftTurnMotorId, SwerveDriveConstants.kBackLeftTurnEncoderId,
             SwerveDriveConstants.kBackLeftTurnEncoderOffset, SwerveDriveConstants.kBackLeftTurnMagnetOffset, SwerveDriveConstants.kBackLeftDriveMotorRev, SwerveDriveConstants.kBackLeftTurnMotorRev);

        this.backRightSwerveModule = new SwerveModule(
            SwerveDriveConstants.kBackRightDriveMotorId, SwerveDriveConstants.kBackRightTurnMotorId, SwerveDriveConstants.kBackRightTurnEncoderId,
             SwerveDriveConstants.kBackRightTurnEncoderOffset, SwerveDriveConstants.kBackRightTurnMagnetOffset, SwerveDriveConstants.kBackRightDriveMotorRev, SwerveDriveConstants.kBackRightTurnMotorRev);

        this.m_gyro = new Pigeon2(SwerveDriveConstants.kGyroId);

        this.turningPIDController = new ProfiledPIDController(
            SwerveDriveConstants.kPModuleTurningController,
            SwerveDriveConstants.kIModuleTurningController,
            SwerveDriveConstants.kDModuleTurningController,
            new TrapezoidProfile.Constraints(
                SwerveDriveConstants.kMaxModuleAngularSpeedDegreesPerSecond,
                SwerveDriveConstants.kMaxModuleAngularAccelDegreesPerSecondSquared), 0.08);

        this.turningPIDController.setTolerance(SwerveDriveConstants.kPostitionToleranceDegrees, SwerveDriveConstants.kVelocityToleranceDegreesPerSec);
        this.turningPIDController.setIntegratorRange(-0.3, 0.3);
        this.m_gyro.reset();

        SmartDashboard.updateValues();
    }

  @Override
  public void periodic() {
    if (desiredChassisSpeeds != null) {
      SwerveModuleState[] desiredStates = SwerveDriveConstants.kDriveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
     
      // If we're not trying to move, we lock the angles of the wheels
      if (desiredChassisSpeeds.vxMetersPerSecond == 0.0 && desiredChassisSpeeds.vyMetersPerSecond == 0.0
          && desiredChassisSpeeds.omegaRadiansPerSecond == 0.0) {
        SwerveModuleState[] currentStates = getModuleStates();
        for(int i = 0; i < currentStates.length; i++) {
            desiredStates[i].angle = new Rotation2d(0);
        }
      }

      // Positive angles should be counter clockwise.
      setModuleStates(desiredStates);
    }
    // Resets the desiredChassisSpeeds to null to stop it from "sticking" to the last states
    desiredChassisSpeeds = null;
    
    updateDashboard();
  }

  private void updateDashboard() {
      SmartDashboard.putNumber("FL MPS", this.frontLeftSwerveModule.getDriveMotorSpeedInMetersPerSecond());
      SmartDashboard.putNumber("FL Angle", this.frontLeftSwerveModule.getTurningEncoderAngleDegrees().getDegrees());

      SmartDashboard.putNumber("FR MPS", this.frontRightSwerveModule.getDriveMotorSpeedInMetersPerSecond());
      SmartDashboard.putNumber("FR Angle", this.frontRightSwerveModule.getTurningEncoderAngleDegrees().getDegrees());

      SmartDashboard.putNumber("BL MPS", this.backLeftSwerveModule.getDriveMotorSpeedInMetersPerSecond());
      SmartDashboard.putNumber("BL Angle", this.backLeftSwerveModule.getTurningEncoderAngleDegrees().getDegrees());
      
      SmartDashboard.putNumber("BR MPS", this.backRightSwerveModule.getDriveMotorSpeedInMetersPerSecond());
      SmartDashboard.putNumber("BR Angle", this.backRightSwerveModule.getTurningEncoderAngleDegrees().getDegrees());

      SmartDashboard.putNumber("Robot Heading in Degrees", this.m_gyro.getAngle()); 

  }

  public SwerveModulePosition[] getModulePositions() {
    SwerveModulePosition[] positions = {
        this.frontLeftSwerveModule.getPosition(),
        this.frontRightSwerveModule.getPosition(),
        this.backLeftSwerveModule.getPosition(),
        this.backRightSwerveModule.getPosition()
    };

    return positions;
  }
  private SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] swerveModuleStates = {
        this.frontLeftSwerveModule.getState(),
        this.frontRightSwerveModule.getState(),
        this.backLeftSwerveModule.getState(),
        this.backRightSwerveModule.getState()
      };

    return swerveModuleStates;
  }
  
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
      desiredStates, SwerveDriveConstants.kMaxSpeedMetersPerSecond);
    this.frontLeftSwerveModule.setDesiredState(desiredStates[0]);
    this.frontRightSwerveModule.setDesiredState(desiredStates[1]);
    this.backLeftSwerveModule.setDesiredState(desiredStates[2]);
    this.backRightSwerveModule.setDesiredState(desiredStates[3]); 
    
    SmartDashboard.putNumber("FL Desired Angle", desiredStates[0].angle.getDegrees());
    SmartDashboard.putNumber("FR Desired Angle", desiredStates[1].angle.getDegrees());
    SmartDashboard.putNumber("BL Desired Angle", desiredStates[2].angle.getDegrees());
    SmartDashboard.putNumber("BR Desired Angle", desiredStates[3].angle.getDegrees());

    SmartDashboard.putNumber("FR Desired Velocity", desiredStates[1].speedMetersPerSecond);
    SmartDashboard.putNumber("FR Actual Velocity", this.frontRightSwerveModule.getState().speedMetersPerSecond);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return SwerveDriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  public double getCurrentChassisSpeeds() {
    ChassisSpeeds currentSpeeds = getChassisSpeeds();
    double linearVeloicity = Math.sqrt((currentSpeeds.vxMetersPerSecond * currentSpeeds.vxMetersPerSecond) * (currentSpeeds.vyMetersPerSecond * currentSpeeds.vyMetersPerSecond));
    return linearVeloicity;
  }

  public Rotation2d getCurrentChassisHeading() {
    ChassisSpeeds currentSpeeds = getChassisSpeeds();
    Rotation2d robotHeading = new Rotation2d(Math.atan2(currentSpeeds.vyMetersPerSecond, currentSpeeds.vxMetersPerSecond));
    return robotHeading;
  }

  /* 
  Start of wrapper for the gyro
  */
  public double getTurnRate() {
    return m_gyro.getRate();
  }
/* 
  public double getRoll() {
    return this.m_gyro.getRoll();
  }
  public double getRollRate() {
    double[] xyzDegPerSec = new double[3];
    this.m_gyro.getRawGyro(xyzDegPerSec);
    return xyzDegPerSec[0];
  }

  public double getPitch() {
    return this.m_gyro.getPitch();
  }

  public double getPitchRate() {
    double[] xyzDegPerSec = new double[3];
    this.m_gyro.getRawGyro(xyzDegPerSec);
    return xyzDegPerSec[1];
  }

  public double getYaw() {
    return this.m_gyro.getRoll();
  }

  public double getYawRate() {
    double[] xyzDegPerSec = new double[3];
    this.m_gyro.getRawGyro(xyzDegPerSec);
    return xyzDegPerSec[2];
  }
 */
  public void zeroHeading() {
    this.m_gyro.reset();
  }

  public double getHeading() {
    return this.m_gyro.getRotation2d().getDegrees();
  }

  public Rotation2d getHeadingInRotation2d() {
    return this.m_gyro.getRotation2d();
  }

  /* 
  End of gyro wrapper
  */

  public void drive(ChassisSpeeds chassisSpeeds) {
    this.desiredChassisSpeeds = chassisSpeeds;
  }
  
  public void stop(){
    drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(0,0
        ,0, new Rotation2d(0)));
  }

  public void lockSwerves(){
    // Crank all the swerve turning motors 45 degrees one way or the other
    SwerveModuleState m_frontLeftLockupState = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    SwerveModuleState m_frontRightLockupState = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    SwerveModuleState m_rearLeftLockupState = new SwerveModuleState(0, Rotation2d.fromDegrees(45));
    SwerveModuleState m_rearRightLockupState = new SwerveModuleState(0, Rotation2d.fromDegrees(-45));
    this.frontLeftSwerveModule.setDesiredState(m_frontLeftLockupState);
    this.frontRightSwerveModule.setDesiredState(m_frontRightLockupState);
    this.backLeftSwerveModule.setDesiredState(m_rearLeftLockupState);
    this.backRightSwerveModule.setDesiredState(m_rearRightLockupState);
  }


}
