// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Autos.AutoMoves;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.autoConstants;
import frc.robot.Constants.swerveConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveDrivetrain;

public class ZeroTo extends SubsystemBase {
  SwerveDrivetrain drivetrain = new SwerveDrivetrain();
  /** Creates a new ZeroTo. */
  public ZeroTo() {}

  @Override
  public void periodic() {
  }
  public static final double kPXController = 1.5;
  public static final double kPYController = 1.5;
  public static final double kPThetaController = 3;

  public Command getCommand(){

  final TrapezoidProfile.Constraints kThetaControllerConstraints =
          new TrapezoidProfile.Constraints(
                  Math.PI,
                  Math.PI/2);
          TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
                1,
                1)
                        .setKinematics(swerveConstants.kinematics);

        // 2. Generate trajectory
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                new Pose2d(0, 0, new Rotation2d(0)),
                List.of(
                        // new Translation2d(1, 0),
                        // new Translation2d(1, -1)
                ),
                new Pose2d(0, -1, Rotation2d.fromDegrees(180)),
                trajectoryConfig);

        // 3. Define PID controllers for tracking trajectory
        PIDController xController = new PIDController(1, 0, 0);
        PIDController yController = new PIDController(1, 0, 0);
        ProfiledPIDController thetaController = new ProfiledPIDController(
                5, 0, 0, kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        // 4. Construct command to follow trajectory
        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory,
                drivetrain::getPose,
                swerveConstants.kinematics,
                xController,
                yController,
                thetaController,
                drivetrain::setDesiredState,
                drivetrain);

        // 5. Add some init and wrap-up, and return everything
        return new SequentialCommandGroup(
                new InstantCommand(() -> drivetrain.resetOdometry(trajectory.getInitialPose())),
                swerveControllerCommand,
                new InstantCommand(() -> drivetrain.stopDrivetrain()));
  }
}
