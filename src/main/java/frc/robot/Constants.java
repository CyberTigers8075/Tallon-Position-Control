// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.subsystems.TalonSub;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;




/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final int kDriverControllerPort = 0;
  public static final int MECH_JOYSTICK = 1;
  public static final int B = 2;
  public static final int X = 3; 
  /* Module Gear Ratios */
  public static final double angleGearRatio = (1.0);
  
/** Radians per Second */
public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot

  /* Motor Inverts */
  public static final InvertedValue angleMotorInvert = InvertedValue.CounterClockwise_Positive;

  /* Angle Encoder Invert */
  public static final SensorDirectionValue cancoderInvert = SensorDirectionValue.CounterClockwise_Positive;

  /* Swerve Current Limiting */
  public static final int angleCurrentLimit = 25;
  public static final int angleCurrentThreshold = 40;
  public static final double angleCurrentThresholdTime = 0.1;
  public static final boolean angleEnableCurrentLimit = true;

  /* Angle Motor PID Values */
  public static final double angleKP = 5;
  public static final double angleKI = 0.0;
  public static final double angleKD = 0.1;

  /* Neutral Modes */
  public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;

   //TODO: This must be tuned to specific motor
  public static final int angleMotor = 1;
  public static final int canCoderID = 0;
  public static final Rotation2d angleOffset = Rotation2d.fromRotations(0.04);
              
            



  public static final class CTREConfigs {
    public static TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();

    public CTREConfigs() {    
      /** Swerve Angle Motor Configurations */
      /* Motor Inverts and Neutral Mode */
      swerveAngleFXConfig.MotorOutput.Inverted = angleMotorInvert;
      swerveAngleFXConfig.MotorOutput.NeutralMode = angleNeutralMode;

      /* Gear Ratio and Wrapping Config */
      swerveAngleFXConfig.Feedback.SensorToMechanismRatio = angleGearRatio;
      swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
          
      /* Current Limiting */
      swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = angleEnableCurrentLimit;
      swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = angleCurrentLimit;

      /* PID Config */
      swerveAngleFXConfig.Slot0.kP = angleKP;
      swerveAngleFXConfig.Slot0.kI = angleKI;
      swerveAngleFXConfig.Slot0.kD = angleKD;
    }
  
  }
}

