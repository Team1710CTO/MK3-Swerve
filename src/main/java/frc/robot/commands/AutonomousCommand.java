package frc.robot.commands;

import java.io.File;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Path;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrivetrain;

public class AutonomousCommand extends CommandBase {
    private int i = 0;
    public static double forwardSpeed;
    public static double strafeSpeed;
    public static double rotation;
    public static List<Double> forwardAr = new ArrayList<>();
    public static List<Double> strafeAr = new ArrayList<>();
    public static List<Double> rotationAr = new ArrayList<>();
    private SwerveDrivetrain drivetrain;

    public AutonomousCommand(SwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        Scanner inputStream = null;
        try {
            String fileName = "movement values 23.csv";
            Path filePath = Filesystem.getDeployDirectory().toPath().resolve(fileName);
            File file = new File(filePath.toString());
            inputStream = new Scanner(file);
            while (inputStream.hasNext()) {
                String data = inputStream.next();
                String[] arr = data.split(",");

                forwardAr.add(Double.parseDouble(arr[0]));
                rotationAr.add(Double.parseDouble(arr[2]));
                strafeAr.add(Double.parseDouble(arr[1]));
            }
        } catch (IOException e) {
            e.printStackTrace();
        } finally {
            if (inputStream != null) {
                inputStream.close();
            }
        }
    }

    @Override
    public void execute() {
        forwardSpeed = forwardAr.get(i);
        strafeSpeed = strafeAr.get(i);
        rotation = rotationAr.get(i);

        drivetrain.drive(forwardSpeed * .2, strafeSpeed * .2, rotation * .6 /* + (-0.01*forwardSpeed) */, false, false);
    }

    @Override
    public boolean isFinished() {
        if (i == forwardAr.size() - 1) {
            return true;
        } else {
            i++;
            return false;
        }
    }
}