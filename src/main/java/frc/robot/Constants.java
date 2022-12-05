package frc.robot;

import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.geometry.Translation2d;

public class Constants {

    public static final class swerveConstants{
        
      // public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
      //   new Translation2d( //front right
      //     -Constants.Chassis_Length/2,
      //     Constants.Chassis_Length/2
      //   ), //front left
      //   new Translation2d(
      //     -Constants.Chassis_Length/2,
      //     -Constants.Chassis_Length/2
      //   ),//back right
      //   new Translation2d(
      //     Constants.Chassis_Length/2,
      //     Constants.Chassis_Length/2
      //   ),//back left
      //   new Translation2d(
      //     Constants.Chassis_Length/2,
      //     -Constants.Chassis_Length/2
      //   )
      // );
      // //this was successful
      public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d( //front right
          Constants.Chassis_Length/2,
          -Constants.Chassis_Length/2
        ), //front left
        new Translation2d(
          Constants.Chassis_Length/2,
          Constants.Chassis_Length/2
        ),//back right
        new Translation2d(
          -Constants.Chassis_Length/2,
          -Constants.Chassis_Length/2
        ),//back left
        new Translation2d(
          -Constants.Chassis_Length/2,
          Constants.Chassis_Length/2
        )
      );
      public static final double driveGearRatio = 6.75;
      public static final double angleGearRatio = 21.43;
    
      public static final double wheelDiameter = Units.inchesToMeters(4.0);
      public static final double wheelCircumference = wheelDiameter * Math.PI;

      public static final class swerveModules{

        //Front Left IDs
        public final static int FRONT_LEFT_DRIVEMOTOR = 2;
        public final static int FRONT_LEFT_TURNMOTOR = 1;
        public final static int FRONT_LEFT_CANCoder = 3;
        //Front Right IDs
        public final static int FRONT_RIGHT_DRIVEMOTOR = 4;
        public final static int FRONT_RIGHT_TURNMOTOR = 5;
        public final static int FRONT_RIGHT_CANCoder = 6;
        //Back Left IDs
        public final static int BACK_LEFT_DRIVEMOTOR = 9;
        public final static int BACK_LEFT_TURNMOTOR = 10;
        public final static int BACK_LEFT_CANCoder = 11;
        //Back Right IDs
        public final static int BACK_RIGHT_DRIVEMOTOR = 7;
        public final static int BACK_RIGHT_TURNMOTOR = 8;
        public final static int BACK_RIGHT_CANCoder = 12;
        
        public final static CANCoder front_left_encoder = new CANCoder(swerveModules.FRONT_LEFT_CANCoder);
        public final static CANCoder front_right_encoder = new CANCoder(swerveModules.FRONT_RIGHT_CANCoder);
        public final static CANCoder back_left_encoder = new CANCoder(swerveModules.BACK_LEFT_CANCoder);
        public final static CANCoder back_right_encoder = new CANCoder(swerveModules.BACK_RIGHT_CANCoder);

        public final static TalonFX front_left_drive = new TalonFX(swerveModules.FRONT_LEFT_DRIVEMOTOR);
        public final static TalonFX front_right_drive = new TalonFX(swerveModules.FRONT_RIGHT_DRIVEMOTOR);
        public final static TalonFX back_left_drive = new TalonFX(swerveModules.BACK_LEFT_DRIVEMOTOR);
        public final static TalonFX back_right_drive = new TalonFX(swerveModules.BACK_RIGHT_DRIVEMOTOR);

        public final static TalonFX front_left_turn =  new TalonFX(swerveModules.FRONT_LEFT_TURNMOTOR);
        public final static TalonFX front_right_turn = new TalonFX(swerveModules.FRONT_RIGHT_TURNMOTOR);
        public final static TalonFX back_left_turn = new TalonFX(swerveModules.BACK_LEFT_TURNMOTOR);
        public final static TalonFX back_right_turn = new TalonFX(swerveModules.BACK_RIGHT_TURNMOTOR);
        }
    }
    public static final class autoConstants{
      public static final double kSlowSpeedMetersPerSecond = 1.7;
        public static final double kSlowAccelerationMetersPerSecondSquared = 2.0;        
      public static final TrajectoryConfig slowSpeedConfig =
                new TrajectoryConfig(
                kSlowSpeedMetersPerSecond,
                kSlowAccelerationMetersPerSecondSquared)    
                .setKinematics(swerveConstants.kinematics)
                        .setStartVelocity(0)
                        .setEndVelocity(0); 

    }

    public final static double Chassis_Length = 0.517;
    public final static double MAX_SPEED = 3;
    public final static double MAX_ACCELERATION = 3;
}
