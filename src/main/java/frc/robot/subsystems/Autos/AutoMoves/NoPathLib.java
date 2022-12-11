// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Autos.AutoMoves;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.autoConstants;
import frc.robot.subsystems.SwerveDrivetrain;

public class NoPathLib extends SubsystemBase {
  /** Creates a new NoPathLib. */
  SwerveDrivetrain drivetrain = new SwerveDrivetrain();
  Trajectory trajectory = TrajectoryGenerator.generateTrajectory(new Pose2d(), 
  List.of(), new Pose2d(0, 1, Rotation2d.fromDegrees(0)), autoConstants.slowSpeedConfig);
  public NoPathLib() {
  }
  public Command getCommand(){
    return new SequentialCommandGroup(
      new InstantCommand(() -> drivetrain.resetOdometry(trajectory.getInitialPose())),
                               drivetrain.createCommandForTrajectory(trajectory).
                               andThen(drivetrain::stopDrivetrain));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
