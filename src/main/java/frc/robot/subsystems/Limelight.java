// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.wpilibj2.command.SubsystemBase;

// public class Limelight extends SubsystemBase {
//   /** Creates a new Limelight. */
//   public Limelight() {}
//   private NetworkTable limelight = NetworkTableInstance.getDefault().getTable("limelight");
//   double xOffset;
//   double yOffset;
//   double area;
//   boolean hasTarget;
//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//   }
//   public void readInput(){
//     xOffset = limelight.getEntry("tx").getDouble(0.0);
//     yOffset = limelight.getEntry("ty").getDouble(0.0);
//     area = limelight.getEntry("ta").getDouble(0.0);
//     hasTarget = limelight.getEntry("tv").getDouble(0) == 1;
//   }
//   public Boolean hasTarget(){
//     return hasTarget;
//   }
// }
