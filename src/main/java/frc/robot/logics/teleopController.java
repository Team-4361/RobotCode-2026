package frc.robot.logics;

import java.util.Optional;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.RobotContainer;
import frc.robot.Settings;




public class teleopController {
    
private CommandJoystick joystickL;
private CommandJoystick joystickR;

  public double xV = 0;
  public double yV = 0;
  public double rV = 0;

  public SlewRateLimiter xfilter = new SlewRateLimiter(4);
  public SlewRateLimiter yfilter = new SlewRateLimiter(4);
  public SlewRateLimiter rfilter = new SlewRateLimiter(4);

    public teleopController(CommandJoystick joyL, CommandJoystick joyR) {
        joystickL = joyL;
        joystickR = joyR;
    }

    public void drivePID() {
 // Read joystick axes
        double xSpeedJoystick = -joystickL.getRawAxis(1); // INVERTED
        if (Math.abs(xSpeedJoystick) < Settings.joystickDeadband) {
            xSpeedJoystick = 0;
        }
        xSpeedJoystick = xfilter.calculate(xSpeedJoystick);

        double ySpeedJoystick = -joystickL.getRawAxis(0); // INVERTED
        if (Math.abs(ySpeedJoystick) < Settings.joystickDeadband) {
            ySpeedJoystick = 0;
        }
        ySpeedJoystick = yfilter.calculate(ySpeedJoystick);
        
        double rSpeedJoystick = -joystickR.getRawAxis(2);
        if (Math.abs(rSpeedJoystick) < Settings.joystickDeadband) {
            rSpeedJoystick = 0;
        }
        rSpeedJoystick = rfilter.calculate(rSpeedJoystick);
        // Flip for red alliance (field-centric coordinate system)
        Optional<Alliance> alliance = DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get().equals(Alliance.Red)) {
            xSpeedJoystick = -xSpeedJoystick;
            ySpeedJoystick = -ySpeedJoystick;
        }

        // Cube inputs for smoother control
        double xInput = Math.pow(xSpeedJoystick, 3);
        double yInput = Math.pow(ySpeedJoystick, 3);
        double rInput = Math.pow(rSpeedJoystick, 3);

        // Convert to velocities
          xV = xInput * RobotContainer.drivebase.getMaximumVelocity();
          yV = yInput * RobotContainer.drivebase.getMaximumVelocity();
          rV = rInput * RobotContainer.drivebase.getMaximumChassisAngularVelocity();
        RobotContainer.drivebase.drive(xV, yV, rV, true);



    }

}
