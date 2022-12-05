// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.reflect.Field;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.subsystems.SwerveDrivetrain;
import frc.robot.subsystems.Autos.AutoMoves.MOVE;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
import frc.robot.subsystems.Autos.AutoMoves.Non_Traj;
public class Robot extends TimedRobot {

  private RobotContainer robotContainer;
  private SwerveDrivetrain mSwerveDrivetrain;
  private MOVE move = new MOVE();
  private Non_Traj nonTraj = new Non_Traj();
  private String autonomous;
  private Command m_autonomousCommand;
  public SendableChooser<String> chooser = new SendableChooser<>();


  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    robotContainer = new RobotContainer();
    chooser.addOption("Non Traj", "nonTraj");
    chooser.addOption("None", "none");
  }
  public String getChooser() {
    return chooser.getSelected();
  }
  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run();
    robotContainer.controller.getLeftY();
  }
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}


  @Override
  public void autonomousInit() {
    // autonomous = mAutoChooser.getAutonomous();
    // System.out.println(autonomous);
    
    m_autonomousCommand = robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
        m_autonomousCommand.schedule();
    }  
  }

  @Override
  public void autonomousPeriodic() {
    // if(autonomous == "MOVE"){
    //   System.out.println("MOVE");
    //   move.routine();
    // }
    // else if(autonomous == "Non_Traj"){
    //   System.out.println("Non");
    //   nonTraj.routine();
    // }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}
}