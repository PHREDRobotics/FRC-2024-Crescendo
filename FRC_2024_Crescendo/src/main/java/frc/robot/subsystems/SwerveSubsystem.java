
package frc.robot.subsystems;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.controls.LogitechPro;
import edu.wpi.first.math.trajectory.Trajectory;

public class SwerveSubsystem extends SubsystemBase {

  public double throttleAdjust(double rawThrottle) {
    return ((-(DriveConstants.kThrottleMax - DriveConstants.kThrottleMin) / 2) * rawThrottle)
        + (DriveConstants.kThrottleMin + DriveConstants.kThrottleMax) / 2;
  }

  private final SwerveModule frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDriveMotorPort,
      DriveConstants.kFrontLeftTurningMotorPort,
      DriveConstants.kFrontLeftDriveEncoderReversed,
      DriveConstants.kFrontLeftTurningEncoderReversed,
      DriveConstants.kFrontLeftModuleChassisAngularOffset,
      DriveConstants.kFrontLeftDriveAbsoluteEncoderReversed,
      DriveConstants.kFrontLeftDriveInverted);

  private final SwerveModule frontRight = new SwerveModule(
      DriveConstants.kFrontRightDriveMotorPort,
      DriveConstants.kFrontRightTurningMotorPort,
      DriveConstants.kFrontRightDriveEncoderReversed,
      DriveConstants.kFrontRightTurningEncoderReversed,
      DriveConstants.kFrontRightModuleChassisAngularOffset,
      DriveConstants.kFrontRightDriveAbsoluteEncoderReversed,
      DriveConstants.kFrontRightDriveInverted);

  public final SwerveModule backLeft = new SwerveModule(
      DriveConstants.kBackLeftDriveMotorPort,
      DriveConstants.kBackLeftTurningMotorPort,
      DriveConstants.kBackLeftDriveEncoderReversed,
      DriveConstants.kBackLeftTurningEncoderReversed,
      DriveConstants.kBackLeftModuleChassisAngularOffset,
      DriveConstants.kBackLeftDriveAbsoluteEncoderReversed,
      DriveConstants.kBackLeftDriveInverted);

  private final SwerveModule backRight = new SwerveModule(
      DriveConstants.kBackRightDriveMotorPort,
      DriveConstants.kBackRightTurningMotorPort,
      DriveConstants.kBackRightDriveEncoderReversed,
      DriveConstants.kBackRightTurningEncoderReversed,
      DriveConstants.kBackRightModuleChassisAngularOffset,
      DriveConstants.kBackRightDriveAbsoluteEncoderReversed,
      DriveConstants.kBackRightDriveInverted);

  private final AHRS gyro = new AHRS(SPI.Port.kMXP);
  // private final SimDeviceSim simGyro = new SimDeviceSim();
  private final Field2d m_field = new Field2d();

  private final LogitechPro m_joyStick;

  // private final SwerveDriveOdometry odometer = new
  // SwerveDriveOdometry(DriveConstants.kDriveKinematics,
  // new Rotation2d(), getModulePositions());

  SwerveDriveOdometry odometer = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics, gyro.getRotation2d(),
      getModulePositions(), new Pose2d(0.0, 0.0, new Rotation2d()));

  // 3
  public PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
  public PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
  public ProfiledPIDController thetaController = new ProfiledPIDController(
      AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);

  public SwerveModulePosition[] getModulePositions() {
    return new SwerveModulePosition[] {
        frontLeft.getPosition(),
        frontRight.getPosition(),
        backLeft.getPosition(),
        backRight.getPosition()
    };
  }

  public SwerveSubsystem(LogitechPro joystick) {
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    m_joyStick = joystick;
    SmartDashboard.putData(m_field);
    new Thread(() -> {
      try {
        Thread.sleep(1000);
        zeroHeading();
        initModules();
      } catch (Exception e) {
      }
    }).start();
  }

  public double getHeading() {
    if (gyro.isMagnetometerCalibrated()) {
      // We will only get valid fused headings if the magnetometer is calibrated
      return gyro.getFusedHeading();
    }

    // We have to invert the angle of the NavX so that rotating the robot
    // counter-clockwise makes the angle increase.
    // if (-gyro.getYaw() >= 0) {
    return gyro.getYaw();

    // } else {
    // return 360 + -gyro.getYaw();
    // }
  }

  public Rotation2d getRotation2d() {
    return Rotation2d.fromDegrees(getHeading());
    // change fromDegrees to fromRadians
  }

  public Pose2d getPose() {
    return odometer.getPoseMeters();
  }

  public Trajectory getTrajectory(Pose2d startPos, Translation2d midPos, Pose2d endPos) {
    // 2
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
        startPos,
        List.of(
            midPos),
        endPos,
        AutoConstants.trajectoryConfig);
    return trajectory;
  }

  public Trajectory getGoToTrajectory(Pose2d targetWaypoint, Pose2d secondPoint) {
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(List.of(), AutoConstants.trajectoryConfig);
    return trajectory;
  }

  public void resetOdometry(Pose2d pose) {
    odometer.resetPosition(getRotation2d(), getModulePositions(), pose);
  }

  public double StepTowards(double _current, double _target) {
    if (Math.abs(_current - _target) <= AutoConstants.kAutoSpeedMetersPerSecond / 2) {
      return 2 * (_target - _current);
    } else if (_current - _target < -AutoConstants.kAutoSpeedMetersPerSecond / 2) {
      return AutoConstants.kAutoSpeedMetersPerSecond;
    } else {
      return -AutoConstants.kAutoSpeedMetersPerSecond;
    }
  }

  public void zeroHeading() {
    // gyro.zeroYaw();
    gyro.reset();
    frontLeft.resetEncoders();
    frontRight.resetEncoders();
    backLeft.resetEncoders();
    backRight.resetEncoders();
    resetOdometry(new Pose2d(0, 0, getRotation2d()));
  }

  // -------------------------------------------------
  // ------------- Update Dashboard ----------------
  @Override
  public void periodic() {
    odometer.update(getRotation2d(), getModulePositions());

    SmartDashboard.putNumber("throttle", m_joyStick.getThrottle());

    SmartDashboard.putString("Robot Heading (Rotation2d)", gyro.getRotation2d().toString());
    SmartDashboard.putNumber("Gameboard/Robot Heading (Degrees)", gyro.getYaw());

    SmartDashboard.putString("Gameboard/Robot Pose:", getPose().getTranslation().toString());

    SmartDashboard.putNumber("Front Left Turning Position", frontLeft.getTurningPosition() * (180 / Math.PI));
    SmartDashboard.putNumber("Front Right Turning Position", frontRight.getTurningPosition() * (180 / Math.PI));
    SmartDashboard.putNumber("Back Left Turning Position", backLeft.getTurningPosition() * (180 / Math.PI));
    SmartDashboard.putNumber("Back Right Turning Position", backRight.getTurningPosition() * (180 / Math.PI));

    m_field.setRobotPose(odometer.getPoseMeters());

    SmartDashboard.putBoolean("Should we blame Hardware/Electrical?", true);
    SmartDashboard.putNumber("Gameboard/Adjusted Throttle", throttleAdjust(m_joyStick.getThrottle()));

  }

  @Override
  public void simulationPeriodic() {
    m_field.setRobotPose(odometer.getPoseMeters());
  }

  public void initModules() {
    SwerveModuleState state = new SwerveModuleState(0, new Rotation2d());
    frontLeft.setDesiredState(state);
    frontRight.setDesiredState(state);
    backLeft.setDesiredState(state);
    backRight.setDesiredState(state);
  }

  public void stopModules() {
    frontLeft.stop();
    frontRight.stop();
    backLeft.stop();
    backRight.stop();
  }
  // state.angle.getRadians());

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
    frontLeft.setDesiredState(desiredStates[0]);
    frontRight.setDesiredState(desiredStates[1]);
    backLeft.setDesiredState(desiredStates[2]);
    backRight.setDesiredState(desiredStates[3]);
  }

}