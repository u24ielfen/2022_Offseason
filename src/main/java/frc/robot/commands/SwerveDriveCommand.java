package frc.robot.commands;

import java.util.Map;

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
    //NetworkTableEntry value = tab.add("kMaxSpeed", 3).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 3)).getEntry();
    //kMaxSpeed = value.getDouble(0.0);
    //NetworkTableEntry value2 = tab.add("kMaxRotation", 3).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 3)).getEntry();
    //kMaxAngle = value2.getDouble(0.0);
    this.drivetrain = drivetrain;
    this.tankDrive = tankDrive;
    addRequirements(drivetrain);

    this.controller = controller;
  }
  double maxControllerNumber = 0.1;

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