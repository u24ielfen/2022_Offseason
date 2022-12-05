// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.swerveConstants.swerveModules;
import frc.robot.Constants.swerveConstants;



public class SwerveDrivetrain extends SubsystemBase {

  public static final double kMaxSpeed = 1;

  public static final double kMaxAngularSpeed = Math.PI;

  SwerveDriveOdometry swerveOdometry;

  SwerveModuleMK3[] modules;

  public SwerveModuleState[] states;

  private Field2d field2d = new Field2d();

  private final static AHRS gyro = new AHRS(SerialPort.Port.kMXP);
  
  
  public SwerveDrivetrain() {
    swerveOdometry = new SwerveDriveOdometry(swerveConstants.kinematics, gyro.getRotation2d());
    zeroGyro();
    SmartDashboard.putData(field2d);
    SmartDashboard.putNumber("Gyro Angle", getGyro().getDegrees());
    SmartDashboard.putBoolean("Run Swerve", true);
    modules = new SwerveModuleMK3[] {
      new SwerveModuleMK3(swerveModules.front_right_drive, swerveModules.front_right_turn, swerveModules.front_right_encoder, Rotation2d.fromDegrees(0)),
      new SwerveModuleMK3(swerveModules.front_left_drive, swerveModules.front_left_turn, swerveModules.front_left_encoder, Rotation2d.fromDegrees(0)),
      new SwerveModuleMK3(swerveModules.back_right_drive, swerveModules.back_right_turn, swerveModules.back_right_encoder, Rotation2d.fromDegrees(0)),
      new SwerveModuleMK3(swerveModules.back_left_drive, swerveModules.back_left_turn, swerveModules.back_left_encoder, Rotation2d.fromDegrees(0))
    };  
    swerveModules.back_left_drive.setInverted(true);
    swerveModules.back_right_drive.setInverted(true);
    swerveModules.front_left_drive.setInverted(true);
    gyro.reset();
  }

  
  public void drive(double xSpeed, double ySpeed, double rotation, boolean fieldRelative) {
    states =
      swerveConstants.kinematics.toSwerveModuleStates(
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                      xSpeed, 
                      ySpeed, 
                      rotation, 
                      gyro.getRotation2d())
                  : new ChassisSpeeds(
                      xSpeed, 
                      ySpeed, 
                      rotation));
                      setDesiredState(states);
                      updateOdometry();
                      field2d.setRobotPose(getPose());
                    }

                    public void stopDrivetrain(){
                      states = swerveConstants.kinematics.toSwerveModuleStates(
                        new ChassisSpeeds(0, 0, 0));  
                        setDesiredState(states);
                      }
  public void setDesiredState(SwerveModuleState[] states){
    SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeed);
    for (int i = 0; i < states.length; i++) {
      SwerveModuleMK3 module = modules[i];
      SwerveModuleState state = states[i];
      if(SmartDashboard.getBoolean("Run Swerve", false)){
        module.setDesiredState(state);
      }
    }
  }

  public Pose2d getPose(){
    return swerveOdometry.getPoseMeters();
  }


  public void resetOdometry(Pose2d pose){
    swerveOdometry.resetPosition(pose, getGyro());
  }
  public void resetFieldPosition(){
    zeroGyro();
    //FIXME: Used to be: new Pose2d(getPose().getTranslation, new Rotation2d()), ...
    swerveOdometry.resetPosition(new Pose2d(getPose().getTranslation(), new Rotation2d()), getGyro());
  }
  
  public void zeroGyro() {
    gyro.reset();
 }
  public SwerveModuleState[] getStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    for(int i = 0; i < modules.length; i ++){
      states[i] = modules[i].getState();
    }
    return states;
  }

  public void updateOdometry(){
    swerveOdometry.update(getGyro(), getStates());
    field2d.setRobotPose(getPose());
  }
  public Rotation2d getGyro(){
    //TODO: Make sure that 360 - etc works
    return Rotation2d.fromDegrees(360- gyro.getYaw());
  }

}