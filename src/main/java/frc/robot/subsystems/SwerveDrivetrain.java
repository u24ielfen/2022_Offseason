// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import java.util.ArrayList;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.kauailabs.navx.frc.AHRS;
// import com.pathplanner.lib.PathPlannerTrajectory;
// import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants.swerveConstants.swerveModules;
import frc.robot.Constants;
import frc.robot.Constants.autoConstants;
import frc.robot.Constants.swerveConstants;

public class SwerveDrivetrain extends SubsystemBase {

  public static final double kMaxSpeed = 1;


  public static final double kMaxAngularSpeed = Math.PI;
  
  SwerveDriveOdometry swerveOdometry;

  SwerveModuleMK3[] modules;

  public SwerveModuleState[] states;

  private Field2d field2d = new Field2d();

  private final static AHRS gyro = new AHRS(SerialPort.Port.kMXP);
  
  public ProfiledPIDController thetaController =
  new ProfiledPIDController(
      5, 0, 0, autoConstants.kThetaControllerConstraints);

      public SwerveDrivetrain() {
    swerveOdometry = new SwerveDriveOdometry(swerveConstants.kinematics, getGyro());
    zeroGyro();
    SmartDashboard.putData(field2d);
    SmartDashboard.putNumber("Gyro Angle", getGyro().getDegrees());
    SmartDashboard.putBoolean("Run Swerve", true);
    modules = new SwerveModuleMK3[] {
      new SwerveModuleMK3(swerveModules.front_left_drive, swerveModules.front_left_turn, swerveModules.front_left_encoder, Rotation2d.fromDegrees(-9.6679)),
      new SwerveModuleMK3(swerveModules.front_right_drive, swerveModules.front_right_turn, swerveModules.front_right_encoder, Rotation2d.fromDegrees(17.929)),
      new SwerveModuleMK3(swerveModules.back_left_drive, swerveModules.back_left_turn, swerveModules.back_left_encoder, Rotation2d.fromDegrees(-38.05664)),
      new SwerveModuleMK3(swerveModules.back_right_drive, swerveModules.back_right_turn, swerveModules.back_right_encoder, Rotation2d.fromDegrees(40.07811))
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
                      getGyro())
                  : new ChassisSpeeds(
                      xSpeed, 
                      ySpeed, 
                      rotation));

    setDesiredState(states);
  }
  
  public void stopDrivetrain(){
    states = swerveConstants.kinematics.toSwerveModuleStates(
      new ChassisSpeeds(0, 0, 0));  
      setDesiredState(states);
    }
    public void setDesiredState(SwerveModuleState[] states){
      
      updateOdometry();
      field2d.setRobotPose(getPose());
    // SmartDashboard.putNumber("Gyro X", gyro.getDisplacementX());
    // SmartDashboard.putNumber("Gyro Y", gyro.getDisplacementY());
    // SmartDashboard.putNumber("Pose X", getPose().getX());
    // SmartDashboard.putNumber("Pose Y", getPose().getY());
    // SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());
    
    // SmartDashboard.putNumber("Gyro Pose X", getGyroPose().getX());
    // SmartDashboard.putNumber("Gyro Pose Y", getGyroPose().getY());
    SwerveDriveKinematics.desaturateWheelSpeeds(states, kMaxSpeed);
    modules[0].setDesiredState(states[0]);
    modules[1].setDesiredState(states[1]);
    modules[2].setDesiredState(states[2]);
    modules[3].setDesiredState(states[3]);
  
}
  public Pose2d getPose(){
    return swerveOdometry.getPoseMeters();
  }
  public Pose2d getGyroPose(){
    return new Pose2d(new Translation2d(gyro.getDisplacementX(), gyro.getDisplacementY()), getGyro());  
  }

  public void resetOdometry(Pose2d pose){
    gyro.setAngleAdjustment( (-1) * pose.getRotation().getDegrees());
    swerveOdometry.resetPosition(pose, getGyro());
  }
  
  public void resetFieldPosition(){
    zeroGyro();
    swerveOdometry.resetPosition(new Pose2d(getPose().getTranslation(), new Rotation2d()), getGyro());
  }
  
 public void zeroGyro(){
   gyro.zeroYaw();
   gyro.resetDisplacement();
  }

  public SwerveModuleState[] getStates(){
    SwerveModuleState[] states = new SwerveModuleState[4];
    for(int i = 0; i < modules.length; i ++){
      states[i] = modules[i].getState();
    }
    SmartDashboard.putNumber("NAV_X", gyro.getDisplacementX());
    SmartDashboard.putNumber("NAV_Y", gyro.getDisplacementY());
    return states;
  }

  public void updateOdometry(){
    swerveOdometry.update(getGyro(), getStates());
  }
  public Rotation2d getGyro(){
    //FIXME: this is new
    // if(gyro.isMagnetometerCalibrated()){
    //   return Rotation2d.fromDegrees(gyro.getFusedHeading());
    // }
    return Rotation2d.fromDegrees(360- gyro.getYaw());
    
  }

  
  public Command createCommandForTrajectory(Trajectory trajectory) {
    return new SwerveControllerCommand(
          trajectory,
          this::getPose,
          swerveConstants.kinematics,
          new PIDController(1, 0, 0),
          new PIDController(1, 0, 0),
          thetaController,
          this::setDesiredState,
          this
    );
  }
  
  // public Command createCommandForPathPlanner(PathPlannerTrajectory trajectory) {
  //   return new PPSwerveControllerCommand(
  //         trajectory,
  //         this::getPose,
  //         swerveConstants.kinematics,
  //         new PIDController(5, 0, 0),
  //         new PIDController(5, 0, 0),
  //         thetaController,
  //         this::setDesiredState,
  //         this
  //   );
  // }

}