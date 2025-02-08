// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Rotation;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.simulation.JoystickSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.TalonSub;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class TalonCommand extends Command {
  /** Creates a new TalonFX. */
  
  private final TalonSub levelconfig;
  private int count;
  Joystick mechJoystick = new Joystick(Constants.MECH_JOYSTICK);

  //private boolean (TODO figure if we need a boolean);
  public TalonCommand(TalonSub lvl) {
    // Use addRequirements() here to declare subsystem dependencies.
    levelconfig = lvl; // it's level NOT 1 v 1!!!!!!!!!! :/
    count = 0;
    addRequirements(levelconfig);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Here");
    System.out.println("A: " + mechJoystick.getRawButton(1));
    System.out.println("B: " + mechJoystick.getRawButton(2));
    System.out.println("X: " + mechJoystick.getRawButton(3));
    System.out.println("Y: " + mechJoystick.getRawButton(4));
    System.out.println("");

    if (mechJoystick.getRawButton(1)){// mechB is pressed, FALCON moves to lvl 1
      TalonSub.level1(-56.13);
    }   else if (mechJoystick.getRawButton(2)){// mechX is pressed, FALCON moves to lvl 2
      TalonSub.level2(-56.13);
    } else if (mechJoystick.getRawButton(3)){// mechX is pressed, FALCON moves to lvl 3
      TalonSub.level3(-56.13);
    } else if (mechJoystick.getRawButton(4)){// mechX is pressed, FALCON moves to floor
      TalonSub.intakeangle(137.837);
    }
  }
  
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}
