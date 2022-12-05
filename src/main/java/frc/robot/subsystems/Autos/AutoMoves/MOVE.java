// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Autos.AutoMoves;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.autoConstants;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Autos.SwerveController;
import frc.robot.subsystems.Autos.WaypointReader;

import java.io.IOException;
import java.nio.file.Path;

public class MOVE extends SubsystemBase {
  /** Creates a new Move. */
  SwerveDrivetrain mSwerveDrivetrain = new SwerveDrivetrain();
  SwerveController path1Controller;
  SwerveController path2Controller;
  private Trajectory path1;
  private Trajectory path2;
  String file_path1 = "paths/Move1.path";
  String file_path2 = "paths/TurnAround.path";
  public MOVE(){
    Path trajectoryPath1 = Filesystem.getDeployDirectory().toPath().resolve(file_path1);
    Path trajectoryPath2 = Filesystem.getDeployDirectory().toPath().resolve(file_path2);
    try{

      TrajectoryGenerator.ControlVectorList control_vectors = WaypointReader.getControlVectors(trajectoryPath1);
      TrajectoryGenerator.ControlVectorList control_vectors2 = WaypointReader.getControlVectors(trajectoryPath2);
      path1 = TrajectoryGenerator.generateTrajectory(control_vectors, autoConstants.slowSpeedConfig);
      path2 = TrajectoryGenerator.generateTrajectory(control_vectors2, autoConstants.slowSpeedConfig);
    }
    catch (IOException io){
      System.out.println("Ummmmmmm...");
    }
    

  }

  public Command routine(){
    path1Controller = new SwerveController(path1);
    path2Controller = new SwerveController(path2);
    return new InstantCommand(() -> mSwerveDrivetrain.resetOdometry(new Pose2d(path1.getInitialPose().getX(), path1.getInitialPose().getY(), Rotation2d.fromDegrees(180))), mSwerveDrivetrain)
    .andThen(path1Controller.getCommand())
    .andThen(path2Controller.getCommand())
    .andThen(new RunCommand(mSwerveDrivetrain::stopDrivetrain, mSwerveDrivetrain));
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
