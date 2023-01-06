// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Limelight extends SubsystemBase {
  
  /** Creates a new Limelight. */
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  double xAngle;
  double yAngle;
  
  double distance;
  boolean hasTarget = false;
  double angleToTarget;
  double xDistance;
  double targetDistance = 0;
  double mountingAngle = 0;
  double limeLightHeight = 0;
  SwerveDrivetrain s_Swerve  = new SwerveDrivetrain();
  public Limelight() {
    turnOn();
  }
  int ledPower = 0;
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    xAngle = table.getEntry("tx").getDouble(0.0);
    yAngle = table.getEntry("ty").getDouble(0.0);
    hasTarget = table.getEntry("tv").getDouble(0) == 1;
    if(hasTarget){
      //y-dist btw robot and target
      double yDistance =  targetDistance - limeLightHeight;
      double angle = Math.toRadians(yAngle) + mountingAngle;
      distance = (yDistance)/Math.tan(angle);

      //x-dist btw robot and targe
       xDistance = Math.tan(xAngle)*distance;

      //X Angle btw robot and target
      angleToTarget = s_Swerve.getPose().getRotation().getRadians() + Math.toRadians(xAngle);
    }
  }
  public double getX(){
    return xAngle;
  }
  public double getY(){
    return yAngle;
  }
  public double getDistanceFromTarget(){
    return distance;
  }
  public double getAngleToTarget(){
    return angleToTarget;
  }
  public boolean hasTarget(){
    return hasTarget;
  }
  public void switchledMode(){
    if(ledPower == 3){
      ledPower = 0;
    }
    if(ledPower == 0){
      ledPower = 3;
    }
    table.getEntry("ledMode").setNumber(ledPower);
  }
  public void turnOn(){
    table.getEntry("ledMode").setNumber(3);
  }
  public void getTelemetry(){
    SmartDashboard.putNumber("X Angle", xAngle);
    SmartDashboard.putNumber("Y Angle", yAngle);
    SmartDashboard.putNumber("AngleToTarget", angleToTarget);
    SmartDashboard.putNumber("X Distance", xDistance);
    SmartDashboard.putNumber("Y Distance", distance);
  }
}

