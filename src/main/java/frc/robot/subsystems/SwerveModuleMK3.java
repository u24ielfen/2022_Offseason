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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  public void setDesiredState(SwerveModuleState desiredState) {
    if(SmartDashboard.getBoolean("Run Swerve", false)){
      driveMotor.setNeutralMode(NeutralMode.Coast);
    }
    else{

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

  }
  
  public double getRawAngle() {
    return canCoder.getAbsolutePosition();
  }
  public void setRawAngle(){
    canCoder.setPosition(0);
  }
  private static final double kWheelRadius = 0.0508;
  private static final int kEncoderResolution = 2048;

  private static final double kDriveGearing = 8.14; // to 1
  private static final double kTurningGearing = 12.8; // to 1

  private static final double kDriveTicksToMeters = 2 * Math.PI * kWheelRadius / kEncoderResolution / kDriveGearing;
  private static final double kDriveTicksToMetersPerSecond = kDriveTicksToMeters * 10;
  private static final double kTurningTicksToRadians = 2 * Math.PI / kEncoderResolution / kTurningGearing;


  public SwerveModuleState getState(){
    // double velocity = falconToMPS(driveMotor.getSelectedSensorVelocity(), swerveConstants.wheelCircumference, swerveConstants.driveGearRatio);
    // Rotation2d angle = Rotation2d.fromDegrees(falconToDegrees(angleMotor.getSelectedSensorPosition(), Constants.swerveConstants.angleGearRatio));
    // return new SwerveModuleState(velocity, angle);
    return new SwerveModuleState(
      driveMotor.getSelectedSensorVelocity() * kDriveTicksToMetersPerSecond,
      new Rotation2d(angleMotor.getSelectedSensorPosition() * kTurningTicksToRadians)
     );
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
      double absolutePosition = degreesToFalcon(canCoder.getAbsolutePosition() - offset.getDegrees(), swerveConstants.angleGearRatio);
      angleMotor.setSelectedSensorPosition(absolutePosition);
      resetToZero();
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
  public void resetToZero(){
    double absolutePosition = degreesToFalcon(Rotation2d.fromDegrees(canCoder.getAbsolutePosition()).getDegrees(), swerveConstants.angleGearRatio);
    angleMotor.setSelectedSensorPosition(absolutePosition);
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