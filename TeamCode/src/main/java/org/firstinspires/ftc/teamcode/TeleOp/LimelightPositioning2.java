package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constance;



@TeleOp
public class LimelightPositioning2 extends OpMode {
    private Limelight3A limelight;
    double limelightXPos;
    double limelightYPos;
    double limelightAngle;
    double pedroPathingXPos;
    double pedroPathingYPos;
    double limePathingAngle;
    double pedroPathingAngle;
    private Follower follower;
    private boolean following = false;
    private final Pose TARGET_LOCATION = new Pose(105.6, 33.33333333333334, Math.toRadians(90));




    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        follower = Constance.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72,90)); //set your starting pose

    }

    @Override
    public void start() {
        limelight.start();

    }

    @Override
    public void loop() {
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose();
            telemetry.addData("Tx", llResult.getTx());
            telemetry.addData("Ty", llResult.getTy());
            telemetry.addData("Ta", llResult.getTa());
            if (botPose != null) {
                limelightXPos = botPose.getPosition().x;
                limelightYPos = botPose.getPosition().y;
                limelightAngle = botPose.getOrientation().getYaw(AngleUnit.DEGREES);
                telemetry.addData("limelightXPos", limelightXPos);
                telemetry.addData("limelightYPos", limelightYPos);
                telemetry.addData("limelightAngle", limelightAngle);
            }
        }

        pedroPathingXPos = (limelightXPos * 39.3700787402) + 72;
        pedroPathingYPos = (limelightYPos * 39.3700787402) + 72;
        limePathingAngle = limelightAngle + 270;
        if (limePathingAngle >= 360) {
            pedroPathingAngle = limePathingAngle - 360;
        } else {
            pedroPathingAngle = limePathingAngle;
        }
        telemetry.addData("pedroPathingXPos", pedroPathingXPos);
        telemetry.addData("pedroPathingYPos", pedroPathingYPos);
        telemetry.addData("pedroPathingAngle", pedroPathingAngle);

        follower.update();
        if (!following) {
            following = true;
            follower.followPath(
                    follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), TARGET_LOCATION))
                            .setLinearHeadingInterpolation(follower.getHeading(), TARGET_LOCATION.minus(follower.getPose()).getAsVector().getTheta())
                            .build()
            );
        }
        else if (following && !follower.isBusy()) {
            following = false;
        }

        follower.setPose(new Pose(pedroPathingXPos, pedroPathingYPos, pedroPathingAngle));
    }

//        private Pose getRobotPoseFromCamera() {
//            return new Pose(0, 0, 0, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
//        }
}

