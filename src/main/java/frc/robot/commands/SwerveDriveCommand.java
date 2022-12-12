package frc.robot.commands;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.swerveConstants.swerveModules;
import frc.robot.subsystems.SwerveDrivetrain;

public class SwerveDriveCommand extends CommandBase {

  private final SwerveDrivetrain drivetrain;
  private final XboxController controller;

  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(3);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(3);

  Boolean tankDrive;
  ShuffleboardTab tab = Shuffleboard.getTab("Swerve Drive");
  double kMaxSpeed = 2;
  double kMaxAngle = 2*Math.PI;
  
  public SwerveDriveCommand(SwerveDrivetrain drivetrain, XboxController controller) {
    
    this.drivetrain = drivetrain;
    this.tankDrive = tankDrive;
    addRequirements(drivetrain);

    this.controller = controller;
  }
  double maxControllerNumber = 0.1;
  public void halfSpeed(){
    if(kMaxSpeed == 2){
      kMaxSpeed = 0.5;
    }
    
  }
  @Override
  public void execute() {
    double xControllerSpeed = controller.getLeftY();
    double yControllerSpeed = controller.getLeftX(); 
    double rotControllerSpeed = controller.getRightX();
    
    xControllerSpeed = Math.abs(xControllerSpeed) > maxControllerNumber ? xControllerSpeed : 0.0;
    
    yControllerSpeed = Math.abs(yControllerSpeed) > maxControllerNumber ? yControllerSpeed : 0.0;
    
    rotControllerSpeed = Math.abs(rotControllerSpeed) > maxControllerNumber ? rotControllerSpeed : 0.0;
    
    final var xSpeed =
      (-xspeedLimiter.calculate(xControllerSpeed)* Math.abs(xspeedLimiter.calculate(xControllerSpeed)))
        * kMaxSpeed;

    final var ySpeed =
      (-yspeedLimiter.calculate(yControllerSpeed) * Math.abs(yspeedLimiter.calculate(yControllerSpeed)))
        * kMaxSpeed;
    final var rot =
      (-rotLimiter.calculate(rotControllerSpeed) * Math.abs(rotLimiter.calculate(rotControllerSpeed)))
        * kMaxAngle;

    boolean fieldRelative = controller.getLeftBumper();
  
    drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative); 
  }
}