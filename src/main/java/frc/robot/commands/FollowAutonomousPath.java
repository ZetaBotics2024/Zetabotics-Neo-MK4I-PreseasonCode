// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// booba

package frc.robot.commands;

import java.util.HashMap;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveDrive.DriveSubsystem;
import frc.robot.subsystems.SwerveDrive.PoseEstimatorSubsystem;

public class FollowAutonomousPath extends CommandBase {
  
  private DriveSubsystem driveSubsystem;
  private PoseEstimatorSubsystem poseEstimatorSubsystem;
  
  public FollowAutonomousPath(DriveSubsystem driveSubsystem, PoseEstimatorSubsystem poseEstimatorSubsystem) {
    this.driveSubsystem = driveSubsystem;
    this.poseEstimatorSubsystem = poseEstimatorSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.driveSubsystem);
    addRequirements(this.poseEstimatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Assuming this is a method in your drive subsystem
public Command followPathCommand(String pathName){
    PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);
    // You must wrap the path following command in a FollowPathWithEvents command in order for event markers to work
    return new FollowPathWithEvents(
        new FollowPathHolonomic(
          path,
          this.poseEstimatorSubsystem::getCurrentPose,
          this.driveSubsystem::getChassisSpeeds,
          this.driveSubsystem::drive,
          new PIDConstants(0), // TODO: Add real PID constants
          new PIDConstants(0),
        
          /*
            path,
            this.poseEstimatorSubsystem::getCurrentPose, // Robot pose supplier
            this.driveSubsystem::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this.driveSubsystem::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your Constants class
                new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
                4.5, // Max module speed, in m/s
                0.4, // Drive base radius in meters. Distance from robot center to furthest module.
                new ReplanningConfig() // Default path replanning config. See the API for the options here
            ),
            this // Reference to this subsystem to set requirements
            */
        ),
        path, // FollowPathWithEvents also requires the path
        this::getPose // FollowPathWithEvents also requires the robot pose supplier
    );
}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  return false;
  }
}