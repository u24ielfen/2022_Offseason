// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Autos.AutoMoves;

import java.time.Instant;
import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.autoConstants;
import frc.robot.Constants.swerveConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Autos.SwerveController;

public class Non_Traj extends SubsystemBase {
  /** Creates a new Non_Traj. */
  
  public static final double kPThetaController = 5;
  SwerveDrivetrain drivetrain = new SwerveDrivetrain();
  public Non_Traj(){
    
  }
  public Command routine(){
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                    ),
            new Pose2d(0.3, 0.3, Rotation2d.fromDegrees(0)),
            autoConstants.slowSpeedConfig);
    TrapezoidProfile.Constraints kThetaControllerConstraints = 
        new TrapezoidProfile.Constraints(Math.PI, 2 / Math.PI);

    PIDController xController = new PIDController(1, 0, 0);
    PIDController yController = new PIDController(1, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, 0, 0, kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            drivetrain::getPose,
            swerveConstants.kinematics,
            xController,
            yController,
            thetaController,
            drivetrain::setDesiredState,
            drivetrain);

    return new SequentialCommandGroup(
            new InstantCommand(() -> drivetrain.resetOdometry(trajectory.getInitialPose())),
            swerveControllerCommand,
            new InstantCommand(() -> drivetrain.stopDrivetrain()));
    }
}
