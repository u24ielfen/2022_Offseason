package frc.robot.subsystems.Autos;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.swerveConstants;
import frc.robot.subsystems.SwerveDrivetrain;

public class SwerveController {
    SwerveDrivetrain mSwerveDrivetrain = new SwerveDrivetrain();
    SwerveControllerCommand swerveControllerCommand;

  public SwerveController (Trajectory trajectory){
      
    
    TrapezoidProfile.Constraints kThetaControllerConstraints = 
        new TrapezoidProfile.Constraints(Math.PI, 2 / Math.PI);
    var thetaController = new ProfiledPIDController(5, 0, 0, kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    
    swerveControllerCommand = new SwerveControllerCommand(trajectory, 
    mSwerveDrivetrain::getPose, 
    swerveConstants.kinematics, 
    new PIDController(1, 0, 0), 
    new PIDController(1, 0, 0), 
    thetaController, 
    mSwerveDrivetrain::setDesiredState, 
    mSwerveDrivetrain);
  }
  public SwerveControllerCommand getCommand(){
    return swerveControllerCommand;
  }
}
