package org.firstinspires.ftc.teamcode.PinionLib;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class DriveToPose {
    Pose2D currentPose;
    Pose2D targetPose;

    PID xControl;
    PID yControl;
    PID hControl;

    DriveToPose(){
        currentPose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES, 0);
        targetPose = new Pose2D(DistanceUnit.INCH, 0,0, AngleUnit.DEGREES, 0);

        xControl = new PID(0.4, 0,0);
        yControl = new PID(0.4, 0,0);
        hControl = new PID(0.05, 0,0);
    }

    public void setCurrentPose(Pose2D Update){
        currentPose = Update;
    }

    public void setTargetPose(Pose2D Update){
        targetPose = Update;
    }

    public boolean driveToPose(Drivetrain yourDrive){
        double xPower = xControl.calculate(currentPose.getX(DistanceUnit.INCH), targetPose.getX(DistanceUnit.INCH));
        double yPower = yControl.calculate(currentPose.getY(DistanceUnit.INCH), targetPose.getY(DistanceUnit.INCH));
        double hPower = hControl.calculate(currentPose.getHeading(AngleUnit.DEGREES), targetPose.getHeading(AngleUnit.DEGREES));

        yourDrive.driveAndTurnFieldRelative(xPower, yPower, hPower, currentPose.getHeading(AngleUnit.RADIANS));
        return false;
    }

    public boolean driveToPose(Pose2D current, Pose2D target, Drivetrain yourDrive){
        setCurrentPose(current);
        setTargetPose(target);

        return driveToPose(yourDrive);
    }
}
