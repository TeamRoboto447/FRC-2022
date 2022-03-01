/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.            */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                   */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean constants. This class should not be used for any other
 * purpose. All constants should be declared globally (i.e. public static). Do
 * not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static Boolean enableCSVLogging = false;
    public static int noLogging = -1, errors = 0, warnings = 1, debug = 2, info = 3;
    
    public static int loggingLevel = errors; // Logging level (-1 = no logging, 0 = errors only, 1 = +warnings, 2 = +debug, 3
                                        // = +info)

    public static final boolean bypassShooterPID = false;

    // Motors
    public static final int leftDrive = 3, // Left Drive Spark Max, CAN ID 3.
            leftDriveB = 4, // Left Drive Spark Max, CAN ID 4.
            rightDrive = 1, // Right Drive Spark Max, CAN ID 1.
            rightDriveB = 2, // Right Drive Spark Max, CAN ID 2.
            climberSparkMax = 5, // Turret Rotation Spark Max, CAN ID 5.
            shooterSparkMaxLeft = 6, // Shooter Left Spark Max, CAN ID 6.
            shooterSparkMaxRight = 7, // Shooter Right Spark Max, CAN ID 7.
            turretSparkMax = 8, // Turret Rotation Spark Max, CAN ID 5.

            intakeTalonSRX = 9, // Intake TalonSRX, CAN ID 9
            indexerTalonSRX = 10, // Indexer TalonSRX, CAN ID 10
            shooterPreloadTalonSRX = 11; // Shooter Preloader TalonSRX, CAN ID 11


    // PCM Channels on board 1 (pneumatics)
    public static final int pneumaticsController = 1, // Pneumatics Board CAN ID 1
            transmissionLow = 0, // Drive transmission Low, PCM channel 0.
            transmissionHigh = 1, // Drive transmission Low, PCM channel 1.
            climberOut = 2, // Climber out, PCM channel 2.
            climberIn = 3, // Climber in, PCM channel 3.
            intakeOut = 4, // Intake extension, PCM channel 4.
            intakeIn = 5;  // Intake retraction, PCM channel 5.

    public static final int indexerFirstPos = 0, // Indexing sensor sensor first position, DIO channel 0.
            fullIndexerSensor = 1; // Full Indexer Sensor , DIO channel 2.

    // Power Distribution Hub
    public static final int PowerDistributionHub = 1; // Power Distribution Hub, CAN ID 1

    //set speeds for intaking balls
    public static final double intakeSpeed = 0.8, indexingSpeed = 0.8;

    //set how long after a ball is shot it will take for vision to reenable.
    public static final double shotInProgressTime = 0.1;

    public static final int // Set safe max current for NEO motors (amps)
    miniNeoSafeAmps = 15, neoSafeAmps = 40;

    public static final double
    // Gear ratios for calculating encoder ticks per rotation
    lowGearRatio = 25.9, highGearRatio = 6.86, thirdStageRatio = 1.14,
            // Wheel diameter for calculating inches per rotation
            wheelDiameter = 6.0, wheelDiameterMeters = 0.1524, shooterWheelDiameter = 6;

    public static final double turretToMoterRatio = 134.24;

    public static final double 
        drivekP = 0.1,
        drivekI = 0, // 1e-4
        drivekD = 0.5,
        drivekIz = 0,
        drivekFF = 0,
        steerkP = 0.0005,
        steerkI = 0,
        steerkD = 0;

    // Shooter PID info
    public static final double shooterkP = 6e-4,
        shooterkI = 2.133096e-03,
        shooterkD = 4.219219e-5,
        shooterkFFm = 1.734411e-04,
        shooterkFFb = -1.372668e-02, 
        shooterIZone = 100,
        shooterSZone = 100,
        speedkM = 0.011525307660967559,
        speedkB = 0.4586392030161885,
        shooterPidIntegralResetTime = 2,
        distanceFromInnerToOuterPort = 29.5 / 12,
        maxInnerPortAjustmentAngle = Math.PI / 4,
        shooterMarginOfError = 60,
        shooterDefaultSpeed = 0.65;

    public static final double cameraHightMeters = 0.9779, targetHightMeters = 2.6035, cameraPitchDegrees = 27.4;

    public static final double distanceLineEqM = 4.168902116976957, distanceLineEqB = -4.08623511110209;

    // Turning PID info
    public static final double
        turretkP = 0.015,
        turretkI = 0.0,
        turretkD = 0.0,
        turretFFm = 0,
        turretFFb = 0,
        turretIZone = 10,
        turretSpinLimit = 90,
        turretMarginOfError = 2,
        staticAimOffset = 0;

    public static final double encoderRes = 1;

    public static final double 
        driveTrainRampTimeToMax = 0.75,
        driveTrainRampMaxTimeDelta = 0.4;

    // Temporary for testing:
    public static final boolean kGyroReversed = true;

    public static final double ksVolts = 0.122;
    public static final double kvVoltSecondsPerMeter = 6.68;
    public static final double kaVoltSecondsSquaredPerMeter = 0.604;
    public static final double kPDriveVel = 0.005;
    public static final double kIDriveVel = 0.0;
    public static final double kDDriveVel = 0.01;

    public static final double kTrackWidthMeters = 0.693702941073319;
    public static final DifferentialDriveKinematics kDriveKinematics = new DifferentialDriveKinematics(
            kTrackWidthMeters);

    public static final double kMaxSpeedMetersPerSecond = 2.1336;
    public static final double kMaxAccelerationMetersPerSecondSquared = 0.5;

    public static final double kRamseteB = 2;
    public static final double kRamseteZeta = 0.7;
}
