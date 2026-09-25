package frc.robot;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.logics.teleopController;

public class Robot extends LoggedRobot   // <-- was TimedRobot
{
  final CommandJoystick joystickL = new CommandJoystick(0);
  final CommandJoystick joystickR = new CommandJoystick(1);
  final CommandXboxController xboxCommandJoystick = new CommandXboxController(3);
  public double xV = 0;
  public double yV = 0;
  public double rV = 0;
  public boolean isAuto = false;
  public SlewRateLimiter xfilter = new SlewRateLimiter(4);
  public SlewRateLimiter yfilter = new SlewRateLimiter(4);
  public SlewRateLimiter rfilter = new SlewRateLimiter(4);
  Thread m_visionThread;
  public teleopController teleopwow;

  private static Robot instance;
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private Timer disabledTimer;

  public Robot()
  {
    instance = this;
  }

  public static Robot getInstance()
  {
    return instance;
  }

  public static final boolean REPLAY_MODE = false; // flip this on manually when replaying

  /**
   * Minimum free space (bytes) required before we trust a target with a
   * WPILOGWriter. Below this, AdvantageKit's file writer can fail to open
   * (or die mid-match) — without a guard, that exception propagates out of
   * robotInit() and kills the whole robot program before RobotContainer
   * (and therefore every subsystem) ever gets constructed.
   */
  private static final long MIN_FREE_BYTES_FOR_LOGGING = 25L * 1024 * 1024; // 25 MB

  @Override
  public void robotInit()
  {
    // ---- AdvantageKit setup: do this before anything else ----
    // Wrapped in try/catch: a logging failure must never stop the rest of
    // robotInit() from running.
    try
    {
      Logger.recordMetadata("ProjectName", "2026Robot");
      Logger.recordMetadata("GitSHA", edu.wpi.first.wpilibj.util.WPILibVersion.Version);

      if (isReal())
      {
        // NT4 never touches disk — add it first so live telemetry still
        // works even if file logging can't start.
        Logger.addDataReceiver(new NT4Publisher());
        safeAddWpilogWriter("/U");
      }
      else if (REPLAY_MODE)
      {
        setUseTiming(false);
        String logPath = LogFileUtil.findReplayLog();
        Logger.setReplaySource(new WPILOGReader(logPath));
        safeAddWpilogWriter(LogFileUtil.addPathSuffix(logPath, "_sim"));
      }
      else
      {
        Logger.addDataReceiver(new NT4Publisher());
        safeAddWpilogWriter("");
      }

      Logger.start();
    }
    catch (Throwable t)
    {
      DriverStation.reportError(
          "AdvantageKit logger setup failed, continuing without it: " + t,
          t.getStackTrace());
    }
    // ------------------------------------------------------------

    m_robotContainer = new RobotContainer();

    teleopwow = new teleopController(joystickL, joystickR, xboxCommandJoystick,
                                      m_robotContainer.getShooter());

    disabledTimer = new Timer();

    if (isSimulation())
    {
      DriverStation.silenceJoystickConnectionWarning(true);
    }
  }

  /**
   * Adds a WPILOGWriter only if the target has enough free space, and never
   * lets a failure here escape — logging is a nice-to-have, not something
   * that should be able to take the robot program down.
   *
   * @param target Path to pass to {@link WPILOGWriter} ("" = onboard default,
   *               "/U" = USB stick, or a specific path for replay output).
   */
  private void safeAddWpilogWriter(String target)
  {
    try
    {
      String checkPath = target.isEmpty() ? "/home/lvuser/logs" : target;
      java.io.File dir = new java.io.File(checkPath);

      long freeBytes = dir.exists() ? dir.getUsableSpace() : Long.MAX_VALUE;
      if (freeBytes < MIN_FREE_BYTES_FOR_LOGGING)
      {
        DriverStation.reportWarning(
            "Skipping WPILOG file logging to '" + target + "' — only "
                + (freeBytes / 1024 / 1024) + " MB free (need "
                + (MIN_FREE_BYTES_FOR_LOGGING / 1024 / 1024) + " MB).",
            false);
        return;
      }

      Logger.addDataReceiver(new WPILOGWriter(target));
    }
    catch (Throwable t)
    {
      DriverStation.reportError(
          "Failed to add WPILOGWriter for '" + target + "', continuing without it: " + t,
          t.getStackTrace());
    }
  }


  @Override
  public void robotPeriodic()
  {
    CommandScheduler.getInstance().run();
    m_robotContainer.updateVision();

    //Not need I think, have to check if this is needed for logging controls or if advantagekit already does it.
   // m_robotContainer.updateControls(); 

    Logger.recordOutput("General/MatchTimeSeconds", DriverStation.getMatchTime());
    Logger.recordOutput("General/BatteryVoltage",
          edu.wpi.first.wpilibj.RobotController.getBatteryVoltage());
  }

  @Override
  public void disabledInit()
  {
    m_robotContainer.setMotorBrake(true);
    disabledTimer.reset();
    disabledTimer.start();
  }

  @Override
  public void disabledPeriodic()
  {
    if (disabledTimer.hasElapsed(Constants.DrivebaseConstants.WHEEL_LOCK_TIME))
    {
      m_robotContainer.setMotorBrake(false);
      disabledTimer.stop();
      disabledTimer.reset();
    }
  }

  @Override
  public void autonomousInit()
  {
    m_robotContainer.setMotorBrake(true);
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    System.out.println("Auto selected: " + m_autonomousCommand);
    if (m_autonomousCommand != null)
    {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
    }
  }

  @Override
  public void autonomousPeriodic()
  {
  }

  @Override
  public void teleopInit()
  {
    if (m_autonomousCommand != null)
    {
      m_autonomousCommand.cancel();
    } else
    {
      CommandScheduler.getInstance().cancelAll();
    }
  }

  @Override
  public void teleopPeriodic()
  {
    teleopwow.drivePID();
  }

  @Override
  public void testInit()
  {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic()
  {
  }

  @Override
  public void simulationInit()
  {
  }

  @Override
  public void simulationPeriodic()
  {
    m_robotContainer.fuelSim.updateSim();
  }
}