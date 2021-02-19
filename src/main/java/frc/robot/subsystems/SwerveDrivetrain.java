// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
public class SwerveDrivetrain extends SubsystemBase {

  public static final double kMaxSpeed = Units.feetToMeters(20); // 13.6 feet per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  public static double feildCalibration = 0;

  public static double frontLeftOffset = 281.689;
  public static double frontRightOffset = 342.86;
  public static double backLeftOffset = 279.1;
  public static double backRightOffset = 89.64;

  public static final int frontLeftDriveId = 1; 
  public static final int frontLeftCANCoderId = 2; 
  public static final int frontLeftSteerId = 3;

  public static final int frontRightDriveId = 4; 
  public static final int frontRightCANCoderId = 5; 
  public static final int frontRightSteerId = 6; 

  public static final int backLeftDriveId = 10; 
  public static final int backLeftCANCoderId = 11; 
  public static final int backLeftSteerId = 12;

  public static final int backRightDriveId = 7; 
  public static final int backRightCANCoderId = 8; 
  public static final int backRightSteerId = 9;   
  /**
   * TODO: These are example values and will need to be adjusted for your robot!
   * Modules are in the order of -
   * Front Left
   * Front Right
   * Back Left
   * Back Right
   * 
   * Positive x values represent moving toward the front of the robot whereas
   * positive y values represent moving toward the left of the robot
   * https://docs.wpilib.org/en/stable/docs/software/kinematics-and-odometry/swerve-drive-kinematics.html#constructing-the-kinematics-object
   */
  private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
    new Translation2d(
      Units.inchesToMeters(10),
      Units.inchesToMeters(10)
    ),
    new Translation2d(
      Units.inchesToMeters(10),
      Units.inchesToMeters(-10)
    ),
    new Translation2d(
      Units.inchesToMeters(-10),
      Units.inchesToMeters(10)
    ),
    new Translation2d(
      Units.inchesToMeters(-10),
      Units.inchesToMeters(-10)
    )
  );

  private final AHRS gyro = new AHRS(SerialPort.Port.kMXP);

  private SwerveModuleMK3[] modules = new SwerveModuleMK3[] {
    new SwerveModuleMK3(new TalonFX(frontLeftDriveId), new TalonFX(frontLeftSteerId), new CANCoder(frontRightCANCoderId), frontLeftOffset), // Front Left
    new SwerveModuleMK3(new TalonFX(frontRightDriveId), new TalonFX(frontRightSteerId), new CANCoder(frontRightSteerId), frontRightOffset), // Front Right
    new SwerveModuleMK3(new TalonFX(backLeftDriveId), new TalonFX(backLeftSteerId), new CANCoder(backLeftCANCoderId), backLeftOffset), // Back Left
    new SwerveModuleMK3(new TalonFX(backRightDriveId), new TalonFX(backRightSteerId), new CANCoder(backRightCANCoderId), backRightOffset)  // Back Right
  };

  public SwerveDrivetrain() {
    gyro.reset(); 
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative, boolean calibrateGyro) {
    
    if(calibrateGyro){
      double feildCalibration = -gyro.getAngle(); //recalibrates gyro offset
    }

    SwerveModuleState[] states =
      kinematics.toSwerveModuleStates(
        fieldRelative
          ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, Rotation2d.fromDegrees((-gyro.getAngle() + feildCalibration)))
          : new ChassisSpeeds(xSpeed, ySpeed, rot));
    SwerveDriveKinematics.normalizeWheelSpeeds(states, kMaxSpeed);
    for (int i = 0; i < states.length; i++) {
      SwerveModuleMK3 module = modules[i];
      SwerveModuleState state = states[i];
      SmartDashboard.putNumber(String.valueOf(i), module.getAngle());
      module.setDesiredState(state);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
