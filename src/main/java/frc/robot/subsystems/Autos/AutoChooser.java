// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoChooser extends SubsystemBase {
  /** Creates a new SmartDashboard. */
  SendableChooser<String> autoChooser;
  String mAutoMode;
  public AutoChooser() {
    autoChooser = new SendableChooser<>();
    autoChooser.addOption("Move", "MOVE");
    autoChooser.addOption("No Pathweaver", "Non_Traj");
    SmartDashboard.putData(autoChooser);
    mAutoMode = autoChooser.getSelected();
  }

  public String getAutonomous(){
    if(mAutoMode != null){
      return mAutoMode;
    }
    return "NOTHING";
  }
}