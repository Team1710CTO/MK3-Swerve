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
    List<Double> forwardAr = new ArrayList<>();
    List<Double> strafeAr = new ArrayList<>();
    List<Double> rotationAr = new ArrayList<>();
    private SwerveDrivetrain drivetrain;

    public AutonomousCommand(SwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {/* Used for testing
        for(double ii = 0; ii <= 0.65; ii += 0.01){
            forwardAr.add(0.0);
            strafeAr.add(0.0);
            rotationAr.add(ii);
        }*/

        Scanner inputStream = null;
        try {
            String fileName = "movement values 4.csv";
            Path filePath = Filesystem.getDeployDirectory().toPath().resolve(fileName);
            File file = new File(filePath.toString());
            // System.out.println(file.exists());
            // System.out.println(file.canRead());
            inputStream = new Scanner(file);
            while (inputStream.hasNext()) {
                String data = inputStream.next();
                String[] arr = data.split(",");

                forwardAr.add(Double.parseDouble(arr[0]));
                strafeAr.add(Double.parseDouble(arr[1]));
                rotationAr.add(Double.parseDouble(arr[2]));
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

        double forwardSpeed = forwardAr.get(i);
        double strafeSpeed = strafeAr.get(i);
        double rotation = rotationAr.get(i);

        drivetrain.drive(forwardSpeed, strafeSpeed, rotation, false, false);
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