
package frc.robot;


import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private RobotContainer robotContainer;
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