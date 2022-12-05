// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.swerveConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Autos.AutoChooser;
import frc.robot.subsystems.Autos.SwerveController;
import frc.robot.subsystems.Autos.AutoMoves.MOVE;
import frc.robot.subsystems.Autos.AutoMoves.Non_Traj;

public class RobotContainer {
  
  public static final double kMaxSpeedMetersPerSecond = 0.25;
  public static final double kMaxAngularSpeedRadiansPerSecond =
          0.2 * Math.PI;
  public static final double kMaxAccelerationMetersPerSecondSquared = Math.PI/4;
  public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 16.00;
  public static final double kPXController = 1;
  public static final double kPYController = 1;
  public static final double kPThetaController = 5;

  public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
          new TrapezoidProfile.Constraints(
                  kMaxAngularSpeedRadiansPerSecond,
                  kMaxAngularAccelerationRadiansPerSecondSquared);

  public final XboxController controller = new XboxController(1);

  private final SwerveDrivetrain drivetrain = new SwerveDrivetrain();
  SwerveController swerveController;
  

  public RobotContainer() {
    configureButtonBindings();
    drivetrain.setDefaultCommand(new SwerveDriveCommand(drivetrain, controller));
  }
  private void configureButtonBindings() {
    new Button(controller::getAButton).whenPressed(() -> drivetrain.resetFieldPosition());
    new Button(controller::getRightBumper).whenPressed(new SwerveDriveCommand(drivetrain, controller));
  }
  public Command getAutonomousCommand(){ // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            kMaxSpeedMetersPerSecond,
            kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(swerveConstants.kinematics);

    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                //     new Translation2d(4/12, -4/12),
                //     new Translation2d(8/12, 0)
                    ),
            new Pose2d(0.3, 0, Rotation2d.fromDegrees(0)),
            trajectoryConfig);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(kPXController, 0, 0);
    PIDController yController = new PIDController(kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
            kPThetaController, 0, 0, kThetaControllerConstraints);
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