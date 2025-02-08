// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

//import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;



public class TalonSub extends SubsystemBase {
  private Rotation2d angleOffset;
  public TalonFX mAngleMotor;
  public DutyCycleEncoder angleEncoder;

  /* angle motor control requests */
  private final  PositionVoltage anglePosition = new PositionVoltage(0);
      
  public TalonSub(){
    this.angleOffset = Constants.angleOffset;
    this.mAngleMotor = new TalonFX(Constants.angleMotor);
    
    /* Angle Encoder Config */
    angleEncoder = new DutyCycleEncoder(Constants.canCoderID);

    /* Angle Motor Config */
    mAngleMotor.getConfigurator().apply(Constants.CTREConfigs.swerveAngleFXConfig);
    resetToAbsolute();
  }
      
  public void setRotation2d(Rotation2d rotation){ // TODO change SwerveModuleState
    mAngleMotor.setControl(anglePosition.withPosition(rotation.getRotations()));
    //mAngleMotor.setControl(anglePosition.withPosition(0));
  }
  
  public Rotation2d getCANcoder(){
    return Rotation2d.fromRotations(angleEncoder.get());
  }  // BOOKMARK: changed from getAbsolutePosition to get()
  
  public void resetToAbsolute(){
    double absolutePosition = getCANcoder().getRotations() - angleOffset.getRotations();
    mAngleMotor.setPosition(absolutePosition);
  }
      
  public Rotation2d getState(){
    return Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble());
  }

  public Rotation2d getPosition(){
    return Rotation2d.fromRotations(mAngleMotor.getPosition().getValueAsDouble());
  }
        
    
      
  /* level degrees */
  // TODO this must be tuned to specific angle
  public void level1(double degrees) {
    mAngleMotor.set(50);
    //-56.13
    Rotation2d rot = new Rotation2d(Math.toRadians(-56.13));
    
    //mAngleMotor.setControl(anglePosition.withPosition(rot.getRotations()));
    //mAngleMotor.setControl(ControlMode.Position(0.15*4096));
    //System.out.println(anglePosition.toString());

  }

  public  Rotation2d level2(double degrees) {
    return new Rotation2d(Math.toRadians(-61));
  }
  public  Rotation2d level3(double degrees) {
    return new Rotation2d(Math.toRadians(123.529));
  }
  public  Rotation2d intakeangle(double degrees) {
    return new Rotation2d(Math.toRadians(137.837));
  }
  
  public  void drive(){
    mAngleMotor.set(50);
  }
}


 

