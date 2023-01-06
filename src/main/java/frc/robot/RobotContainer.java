
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
import frc.robot.Constants.autoConstants;
import frc.robot.Constants.swerveConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Autos.AutoMoves.ZeroTo;

public class RobotContainer {

  public final XboxController controller = new XboxController(1);

  private final SwerveDrivetrain drivetrain = new SwerveDrivetrain();
  SwerveDriveCommand swerveController = new SwerveDriveCommand(drivetrain, controller);
  ZeroTo autoTesting = new ZeroTo();


  public RobotContainer() {
    configureButtonBindings();
    drivetrain.setDefaultCommand(swerveController);
  }
  private void configureButtonBindings() {
    new Button(controller::getAButton).whenPressed(() -> drivetrain.resetFieldPosition());
    new Button(controller::getRightBumper).whenPressed(() -> swerveController.halfSpeed());
  }
  public Command getAutonomousCommand(){
          return autoTesting.getCommand();
  }
}