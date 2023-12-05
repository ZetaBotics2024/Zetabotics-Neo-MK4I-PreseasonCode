package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveDriveConstants;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.subsystems.SwerveDrive.PoseEstimatorSubsystem;


public class GoToPose extends CommandBase {

    private final DriveSubsystem m_driveSubsystem;
    private final PoseEstimatorSubsystem poseEstimatorSystem;
    FollowPathWithEvents eventPath;
    private SwerveAutoBuilder autoBuilder;
    private CommandBase fullAuto = Commands.none();
    private Pose2d endPose;
  
  
    public GoToPose(
        DriveSubsystem m_driveSubsystem, PoseEstimatorSubsystem poseEstimatorSubsystem,
        Pose2d endPose) {
      this.m_driveSubsystem = m_driveSubsystem;
      this.poseEstimatorSystem = poseEstimatorSubsystem;
      this.endPose = endPose;
    }
  
    @Override
    public void initialize() {
        AutoBuilder.configureHolonomic(
        this.poseEstimatorSystem::getCurrentPose, // Robot pose supplier
        this.poseEstimatorSystem::setCurrentPose, // Method to reset odometry (will be called if your auto has a starting pose)
        this.m_driveSubsystem::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        this.m_driveSubsystem::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
        new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
            new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
            4.5, // Max module speed, in m/s
            0.4, // Drive base radius in meters. Distance from robot center to furthest module.
            new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        this // Reference to this subsystem to set requirements
    );

    FollowPathWithEvents(
        new FollowPathHolonomic(
            path,
            this.poseEstimatorSystem::getCurrentPose, // Robot pose supplier
            this.poseEstimatorSystem::setCurrentPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this.m_driveSubsystem::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this.m_driveSubsystem::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                4.5, // Max module speed, in m/s
                0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
        ),
        this // Reference to this subsystem to set requirements
        ),
        path, // FollowPathWithEvents also requires the path
        this::getPose // FollowPathWithEvents also requires the robot pose supplier
    );

    }

   
    @Override
    public void execute() {
      fullAuto.execute();
    }
  
    @Override
    public void end(boolean interrupted) {
      fullAuto.end(interrupted);
    }
  
    @Override
    public boolean isFinished() {
      return fullAuto.isFinished();
    }
  }