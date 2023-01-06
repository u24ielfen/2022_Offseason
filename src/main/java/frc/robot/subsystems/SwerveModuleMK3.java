package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Constants.swerveConstants;
import frc.robot.Constants.swerveConstants.swerveModules;

public class SwerveModuleMK3 {

  private static final double kDriveP = 15.0;
  private static final double kDriveI = 0.01;
  private static final double kDriveD = 0.1;
  private static final double kDriveF = 0.2;

  private static final double kAngleP = 1.0;
  private static final double kAngleI = 0.0;
  private static final double kAngleD = 0.0;

  //TODO: Change?
  Rotation2d offset;

  double lastAngle;
  //FIXME: This changed
  private static double kEncoderTicksPerRotation = 4096;

  private TalonFX driveMotor;
  private TalonFX angleMotor;
  private CANCoder canCoder;

  public SwerveModuleMK3(TalonFX driveMotor, TalonFX angleMotor, CANCoder canCoder, Rotation2d offset) {
    this.driveMotor = driveMotor;
    this.angleMotor = angleMotor;
    this.canCoder = canCoder;
    this.offset = offset;
    configAngleTalon();
    configDriveTalon();

    CANCoderConfiguration canCoderConfiguration = new CANCoderConfiguration();
    canCoderConfiguration.magnetOffsetDegrees = offset.getDegrees();
    canCoder.configAllSettings(canCoderConfiguration);

    lastAngle = getState().angle.getDegrees();
  }

  public Rotation2d getAngle() {
       return Rotation2d.fromDegrees(canCoder.getPosition());
  }
  SwerveModuleState desiredState;
  public void setDesiredState(SwerveModuleState desiredState) {
    this.desiredState = desiredState;
    Rotation2d currentRotation = getAngle();
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, currentRotation);
    double moveMotor = state.speedMetersPerSecond;
    double angleMove = state.angle.getDegrees();
    if(moveMotor <= 0.1){
      angleMove = 0;
    }
    double distanceForAngle = angleMove/(360*(Constants.swerveConstants.angleGearRatio * 2048));
    angleMotor.set(TalonFXControlMode.Position, distanceForAngle);

  }
  
  public double getRawAngle() {
    return canCoder.getAbsolutePosition();
  }
  public void setRawAngle(){
    canCoder.setPosition(0);
  }

  public SwerveModuleState getState(){
    double sensorStuff = driveMotor.getSelectedSensorVelocity() * Constants.swerveConstants.driveGearRatio * 2048/600;
    double metersPerS = sensorStuff * Constants.swerveConstants.wheelCircumference / 60;
    double angleSensorStuff = angleMotor.getSelectedSensorPosition() * (360/Constants.swerveConstants.angleGearRatio * 2048);
    Rotation2d angles = Rotation2d.fromDegrees(angleSensorStuff);
    return new SwerveModuleState(metersPerS, angles);
  }
  public void resetWheelPosition(){
    canCoder.setPosition(0);
    angleMotor.setSelectedSensorPosition(0);
  }
  
  public void configAngleTalon(){
    TalonFXConfiguration angleTalonFXConfiguration = new TalonFXConfiguration();
    angleMotor.configFactoryDefault();
    angleTalonFXConfiguration.slot0.kP = kAngleP;
    angleTalonFXConfiguration.slot0.kI = kAngleI;
    angleTalonFXConfiguration.slot0.kD = kAngleD;
    
    angleTalonFXConfiguration.remoteFilter0.remoteSensorDeviceID = canCoder.getDeviceID();
    angleTalonFXConfiguration.remoteFilter0.remoteSensorSource = RemoteSensorSource.CANCoder;
    angleTalonFXConfiguration.primaryPID.selectedFeedbackSensor = FeedbackDevice.RemoteSensor0;
    angleMotor.configAllSettings(angleTalonFXConfiguration);
    resetAngleTalon();
  }
  public void resetAngleTalon(){
      angleMotor.setSelectedSensorPosition(0);
  }
  public void configDriveTalon(){
    TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();
    driveMotor.configFactoryDefault();
    driveMotor.setSelectedSensorPosition(0);
    driveMotor.setNeutralMode(NeutralMode.Brake);
    driveTalonFXConfiguration.slot0.kP = kDriveP;
    driveTalonFXConfiguration.slot0.kI = kDriveI;
    driveTalonFXConfiguration.slot0.kD = kDriveD;
    driveTalonFXConfiguration.slot0.kF = kDriveF;
  
    driveMotor.configAllSettings(driveTalonFXConfiguration);
  }
  
}