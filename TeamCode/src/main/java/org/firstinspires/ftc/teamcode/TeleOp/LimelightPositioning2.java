package org.firstinspires.ftc.teamcode.TeleOp;

import com.pedropathing.follower.Follower;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.pedroPathing.Constance;



@TeleOp
public class LimelightPositioning2 extends OpMode {
    private Limelight3A limelight;
    private GoBildaPinpointDriver pinpoint;
    double limelightXPos;
    double limelightYPos;
    double limelightAngle;
    double pedroPathingXPos;
    double pedroPathingYPos;
    double limePathingAngle;
    double pedroPathingAngle;
    boolean iSawTheSign = false;
    private Follower follower;
    private IMU imu;
    private boolean following = false;
    private final Pose TARGET_LOCATION = new Pose(105.6, 33.33333333333334, Math.toRadians(90));




    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
//        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        limelight.pipelineSwitch(0);
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));
        follower = Constance.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72,72,0));
//        pinpoint.setOffsets(-0.787, 1.1, DistanceUnit.INCH);
//        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
//
//        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
//        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
//                GoBildaPinpointDriver.EncoderDirection.FORWARD);
//        pinpoint.resetPosAndIMU();

    }

    @Override
    public void start() {
        limelight.start();
        follower.startTeleopDrive(true);

    }

    @Override
    public void loop() {
//        pinpoint.update();
//        Pose2D pose2D = pinpoint.getPosition();
        follower.update();
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        telemetry.addData("IMU Yaw", orientation.getYaw());
        limelight.updateRobotOrientation(orientation.getYaw());
        LLResult llResult = limelight.getLatestResult();
        if (llResult != null && llResult.isValid()) {
            Pose3D botPose = llResult.getBotpose_MT2();
//            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, pedroPathingYPos, pedroPathingXPos, AngleUnit.DEGREES, orientation.getYaw()));
            if (botPose != null) {
                iSawTheSign = true;
                limelightXPos = botPose.getPosition().x * 39.3700787402;
                limelightYPos = botPose.getPosition().y * 39.3700787402;
                limelightAngle = botPose.getOrientation().getYaw(AngleUnit.DEGREES);
                telemetry.addData("limelightXPos", limelightXPos);
                telemetry.addData("limelightYPos", limelightYPos);
                telemetry.addData("limelightAngle", limelightAngle);
            }
        } else {
            iSawTheSign = false;
        }

        limePathingAngle = orientation.getYaw() + 270;
        if (limePathingAngle >= 360) {
            pedroPathingAngle = limePathingAngle - 360;
        } else {
            pedroPathingAngle = limePathingAngle;
        }


        pedroPathingXPos = limelightYPos + 72;
        pedroPathingYPos = -limelightXPos + 72;




        if (!following) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true
            );

        }


        if (gamepad1.aWasPressed()) {
            follower.followPath(
                    follower.pathBuilder()
                            .addPath(new BezierLine(follower.getPose(), TARGET_LOCATION))
                            .setLinearHeadingInterpolation(follower.getHeading(), TARGET_LOCATION.getHeading())
                            //.setLinearHeadingInterpolation(follower.getHeading(), TARGET_LOCATION.minus(follower.getPose()).getAsVector().getTheta())
                            .build()
            );
            following = true;
        }
        if (following && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            following = false;
        }
//        if (following && !follower.isBusy()) {
//            following = false;
//        }
        if(iSawTheSign && !following){
            follower.setPose(new Pose(pedroPathingXPos, pedroPathingYPos, Math.toRadians(pedroPathingAngle)));
        }

        telemetry.addData("pedroPathingXPos", pedroPathingXPos);
        telemetry.addData("pedroPathingYPos", pedroPathingYPos);
        telemetry.addData("pedroPathingAngle", pedroPathingAngle);
        telemetry.addData("followerX", String.format("%.2f", follower.getPose().getX()));
        telemetry.addData("followerY", String.format("%.2f", follower.getPose().getY()));
        telemetry.addData("followerHeading", String.format("%.2f", Math.toDegrees(follower.getPose().getHeading())));
        telemetry.addData("followering", following);
//        telemetry.addData("pinpointX", pose2D.getY(DistanceUnit.INCH));
//        telemetry.addData("pinpointY", pose2D.getX(DistanceUnit.INCH));
//        telemetry.addData("pinpointAngle", pose2D.getHeading(AngleUnit.DEGREES));


    }

//        private Pose getRobotPoseFromCamera() {
//            return new Pose(limelightXPos, limelightYPos, limelightAngle, FTCCoordinates.INSTANCE).getAsCoordinateSystem(PedroCoordinates.INSTANCE);
//        }
}