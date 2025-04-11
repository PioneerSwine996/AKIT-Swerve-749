// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.subsystems.drive;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.util.PIDF;
import frc.robot.util.Util;

public class DriveConstants {
  public static final ModuleConfig moduleConfig = switch (Constants.currentMode) {
    case SIM -> new ModuleConfig(
            PIDF.ofPDSV(0.05, 0.0, 0.02522, 0.14115),
            PIDF.ofPD(8, 0),
            Mk4iGearRatios.L2,
            Mk4iGearRatios.TURN,
            false,
            false,
            false,
            120,
            60
    );
    case REAL -> new ModuleConfig(
            PIDF.ofPDSVA(
                    0.0, 0.0,
                    // FL + FR + BL + BR
                    Util.average(0.024319, 0.094701 /* , [erroneous], [erroneous] */),
                    Util.average(0.13551, 0.13733, 0.13543, 0.14087),
                    Util.average(0.0065694, 0.0054738, /* [erroneous], */ 0.0091241)
            ),
            PIDF.ofPD(0.5, 0.0),
            Mk4iGearRatios.L2,
            Mk4iGearRatios.TURN,
            false,
            false,
            false,
            60,
            60
    );
    case REPLAY -> null;
  };
  public static final double maxSpeedMetersPerSec = 4.60248;
  public static final double odometryFrequency = 100.0; // Hz
  public static final double trackWidth = Units.inchesToMeters(26.5);
  public static final double wheelBase = Units.inchesToMeters(26.5);
  public static final double driveBaseRadius = Math.hypot(trackWidth / 2.0, wheelBase / 2.0);
  public static final Translation2d[] moduleTranslations =
      new Translation2d[] {
        new Translation2d(trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(trackWidth / 2.0, -wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, wheelBase / 2.0),
        new Translation2d(-trackWidth / 2.0, -wheelBase / 2.0)
      };

  // Zeroed rotation values for each module, see setup instructions
  public static final Rotation2d frontLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d frontRightZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backLeftZeroRotation = new Rotation2d(0.0);
  public static final Rotation2d backRightZeroRotation = new Rotation2d(0.0);

  // Device CAN IDs
  public static final int pigeonCanId = 13;

  public static final int frontLeftDriveCanId = 5;
  public static final int backLeftDriveCanId = 7;
  public static final int frontRightDriveCanId = 3;
  public static final int backRightDriveCanId = 1;

  public static final int frontLeftTurnCanId = 6;
  public static final int backLeftTurnCanId = 8;
  public static final int frontRightTurnCanId = 4;
  public static final int backRightTurnCanId = 2;

  public static final int frontLeftCANCoderCanId = 9;
  public static final int backLeftCANCoderCanId = 10;
  public static final int frontRightCANCoderCanId = 11;
  public static final int backRightCANCoderCanId = 12;
  // Drive motor configuration
  public static final int driveMotorCurrentLimit = 50;
  public static final double wheelRadiusMeters = Units.inchesToMeters(2);
//  public static final double driveMotorReduction = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  public static final DCMotor driveGearbox = DCMotor.getNEO(1);

  // Drive encoder configuration
  public static final double driveEncoderPositionFactor =
      2 * Math.PI / moduleConfig.driveGearRatio(); // Rotor Rotations -> Wheel Radians
  public static final double driveEncoderVelocityFactor =
      (2 * Math.PI) / 60.0 / moduleConfig.driveGearRatio(); // Rotor RPM -> Wheel Rad/Sec

  // Drive PID configuration
//  public static final double driveKp = 0.0;
//  public static final double driveKd = 0.0;
//  public static final double driveKs = 0.0;
//  public static final double driveKv = 0.0;
//  public static final double driveSimP = 1.0;
//  public static final double driveSimD = 0.0;
  public static final double driveSimKs = 0.02522;
  public static final double driveSimKv = 0.14115;

  // Turn motor configuration
//  public static final boolean turnInverted = false;
//  public static final int turnMotorCurrentLimit = 35;
//  public static final double turnMotorReduction = (150.0 / 7.0);
  public static final DCMotor turnGearbox = DCMotor.getNEO(1);

  // Turn encoder configuration
//  public static final boolean turnEncoderInverted = true;
//  public static final double turnEncoderPositionFactor = 2 * Math.PI; // Rotations -> Radians
//  public static final double turnEncoderVelocityFactor = (2 * Math.PI) / 60.0; // RPM -> Rad/Sec

  // Turn PID configuration
//  public static final double turnKp = 0.005;
//  public static final double turnKd = 0;
//  public static final double turnSimP = 4.5;
//  public static final double turnSimD = 0.0;
//  public static final double turnPIDMinInput = 0; // Radians
//  public static final double turnPIDMaxInput = 2 * Math.PI; // Radians

  public record ModuleConfig(
          PIDF driveGains,
          PIDF turnGains,
          double driveGearRatio,
          double turnGearRatio,
          boolean turnInverted,
          boolean driveInverted,
          boolean encoderInverted,
          int driveCurrentLimit, // AKA current that causes wheel slip
          int turnCurrentLimit
  ) {
  }
  private static class Mk4iGearRatios {
    public static final double L2 = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
    public static final double L3 = (50.0 / 14.0) * (16.0 / 28.0) * (45.0 / 15.0);

    public static final double TURN = (150.0 / 7.0);
  }
}
