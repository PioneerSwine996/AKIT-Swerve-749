package frc.robot.subsystems.drive;

import frc.robot.util.PIDF;
import frc.robot.util.network.LoggedTunableNumber;

import static frc.robot.subsystems.drive.DriveConstants.*;

public class DriveTuning {
    public static final LoggedTunableNumber characterizationSpeedRadPerSec = new LoggedTunableNumber("Drive/Wheel Radius Characterization Rotation Speed (rad per sec)", 1.0);

    public static final PIDF.Tunable moduleDriveGainsTunable = moduleConfig.driveGains().tunable("Drive/ModuleDrive");
    public static final PIDF.Tunable moduleTurnGainsTunable = moduleConfig.turnGains().tunable("Drive/ModuleTurn");

//    public static final PIDF.Tunable moveToLinearTunable = moveToLinear.tunable("Drive/MoveToLinear");
//    public static final PIDF.Tunable moveToAngularTunable = moveToAngular.tunable("Drive/MoveToAngular");
}
