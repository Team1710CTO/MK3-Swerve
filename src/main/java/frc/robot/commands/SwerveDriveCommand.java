package frc.robot.commands;

import java.io.BufferedWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.SlewRateLimiter;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class SwerveDriveCommand extends CommandBase {

  private final SwerveDrivetrain drivetrain;
  private final XboxController controller;

  // Slew rate limiters to make joystick inputs more gentle; 1/3 sec from 0 to 1.
  private final SlewRateLimiter xspeedLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter yspeedLimiter = new SlewRateLimiter(6);
  private final SlewRateLimiter rotLimiter = new SlewRateLimiter(6);

  List<Double> xspeedAr = new ArrayList<>();
  List<Double> yspeedAr = new ArrayList<>();
  List<Double> rotationAr = new ArrayList<>();

  private static boolean recordMode = false;

  public SwerveDriveCommand(SwerveDrivetrain drivetrain, XboxController controller) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);

    this.controller = controller;
  }
  @Override
  public void initialize(){
    drivetrain.gyro.reset();
  }

  @Override
  public void execute() {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    final var xSpeed = -xspeedLimiter.calculate(controller.getY(GenericHID.Hand.kLeft)) * SwerveDrivetrain.kMaxSpeed;

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    final var ySpeed = -yspeedLimiter.calculate(controller.getX(GenericHID.Hand.kLeft)) * SwerveDrivetrain.kMaxSpeed;

    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    final var rot = -rotLimiter.calculate(controller.getX(GenericHID.Hand.kRight)) * SwerveDrivetrain.kMaxAngularSpeed;

    //boolean calibrate = controller.getBumper(GenericHID.Hand.kLeft);

    SmartDashboard.putNumber("Forward", xSpeed);
    SmartDashboard.putNumber("Strafe", ySpeed);
    SmartDashboard.putNumber("Rotation", rot);

    if (recordMode) {
      xspeedAr.add(xSpeed);
      yspeedAr.add(ySpeed);
      rotationAr.add(rot);
    }

    drivetrain.drive(xSpeed * .2, ySpeed * .2, (rot * .6) /*+ (0.05*xSpeed)*/, false, false);

  }

  @Override
  public void end(boolean interrupted) {
    if (recordMode) {
      try {
        BufferedWriter writer = Files.newBufferedWriter(Paths.get("/home/lvuser/movement values 24.csv"));

        // write all records
        for (int i = 0; xspeedAr.size() > i; i++) {
          writer.write(Double.toString(xspeedAr.get(i)) + ",");
          writer.write(Double.toString(yspeedAr.get(i)) + ",");
          writer.write(Double.toString(rotationAr.get(i)));
          writer.newLine();
        }

        writer.close();
        //SmartDashboard.putBoolean("Write Status", true);

      } catch (IOException ex) {
        ex.printStackTrace();
        //SmartDashboard.putBoolean("Write Status", false);
      }
    }
  }
}