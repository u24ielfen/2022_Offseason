package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.Constants.swerveConstants;
import frc.robot.Constants.swerveConstants.swerveModules;
import frc.robot.lib.CTREModuleState;

public class SwerveModuleMK3 {

  private static final double kDriveP = 15.0;
  private static final double kDriveI = 0.01;
  private static final double kDriveD = 0.1;
  private static final double kDriveF = 0.2;

  private static final double kAngleP = 1.0;
  private static final double kAngleI = 0.0;
  private static final double kAngleD = 0.0;

  //TODO: Change?
  double angleOffset = 0;

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

  public void setDesiredState(SwerveModuleState desiredState) {
    Rotation2d currentRotation = getAngle();
    
    //SwerveModuleState state = CTREModuleState.optimize(desiredState, getState().angle);
    SwerveModuleState state2 = SwerveModuleState.optimize(desiredState, currentRotation);
    Rotation2d rotationDelta = state2.angle.minus(currentRotation);

    double deltaTicks = (rotationDelta.getDegrees() / 360) * kEncoderTicksPerRotation;
    double currentTicks = canCoder.getPosition() / canCoder.configGetFeedbackCoefficient();
    double desiredTicks = currentTicks + deltaTicks;
    
    double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveDrivetrain.kMaxSpeed * 0.01)) ? lastAngle : desiredState.angle.getDegrees();
    lastAngle = angle;

    //FIXME: This changed (wasn't /4)
    angleMotor.set(TalonFXControlMode.Position, desiredTicks);

    //angleMotor.set(ControlMode.Position, degreesToFalcon(angle, swerveConstants.angleGearRatio));
    driveMotor.set(TalonFXControlMode.PercentOutput, state2.speedMetersPerSecond / SwerveDrivetrain.kMaxSpeed);

  }
  
  public double getRawAngle() {
    return canCoder.getAbsolutePosition();
  }
  public void setRawAngle(){
    canCoder.setPosition(0);
  }

  public SwerveModuleState getState(){
    double velocity = falconToMPS(driveMotor.getSelectedSensorPosition(), swerveConstants.wheelCircumference, swerveConstants.driveGearRatio);
    Rotation2d angle = Rotation2d.fromDegrees(falconToDegrees(angleMotor.getSelectedSensorPosition(), Constants.swerveConstants.angleGearRatio));
    return new SwerveModuleState(velocity, angle);
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
      double absolutePosition = degreesToFalcon(canCoder.getAbsolutePosition() - angleOffset, swerveConstants.angleGearRatio);
      angleMotor.setSelectedSensorPosition(absolutePosition);
  }
  public void configDriveTalon(){
    TalonFXConfiguration driveTalonFXConfiguration = new TalonFXConfiguration();
    driveMotor.configFactoryDefault();
    driveMotor.setSelectedSensorPosition(0);
    driveTalonFXConfiguration.slot0.kP = kDriveP;
    driveTalonFXConfiguration.slot0.kI = kDriveI;
    driveTalonFXConfiguration.slot0.kD = kDriveD;
    driveTalonFXConfiguration.slot0.kF = kDriveF;
  
    driveMotor.configAllSettings(driveTalonFXConfiguration);
  }

  //CONVERSIONS:
  public static double falconToMPS(double velocitycounts, double circumference, double gearRatio){
    double wheelRPM = falconToRPM(velocitycounts, gearRatio);
    double wheelMPS = (wheelRPM * circumference) / 60;
    return wheelMPS;
  }
  
  public static double falconToRPM(double velocityCounts, double gearRatio) {
    double motorRPM = velocityCounts * (600.0 / 2048.0);        
    double mechRPM = motorRPM / gearRatio;
    return mechRPM;
  }
  
  public static double falconToDegrees(double counts, double gearRatio) {
    return counts * (360.0 / (gearRatio * 2048.0));
  }

  public static double degreesToFalcon(double degrees, double gearRatio) {
    double ticks =  degrees / (360.0 / (gearRatio * 2048.0));
    return ticks;
  }  
}