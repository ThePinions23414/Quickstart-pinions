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
public class LimelightPositioning extends OpMode {
    private Limelight3A limelight;
    double limelightXPos;
    double limelightYPos;
    double limelightAngle;
    double pedroPathingXPos;
    double pedroPathingYPos;
    double limePathingAngle;
    double pedroPathingAngle;

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        IDLE,
        LIMEPATHINGPATH
    }
    PathState pathState;

    private final Pose currentPose = new Pose(pedroPathingXPos, pedroPathingYPos, Math.toRadians(pedroPathingAngle));
    private final Pose limePathingPose = new Pose(105.6, 33.33333333333334, Math.toRadians(90));


    private PathChain limePath;

    public void buildPaths() {
        limePath = follower.pathBuilder()
                .addPath(new BezierLine(currentPose, limePathingPose))
                .setLinearHeadingInterpolation(currentPose.getHeading(), limePathingPose.getHeading())
                .build();

    }

    public void statePathUpdate() {
        switch (pathState) {
            case IDLE:
                if (gamepad2.a){
                    setPathState(PathState.LIMEPATHINGPATH);
                }
                break;

            case LIMEPATHINGPATH:
                follower.followPath(limePath, true);
                if(!follower.isBusy()){
                    setPathState(PathState.IDLE);
                }
                break;

            default:
                telemetry.addLine("No State Commanded");
                break;
        }


    }
    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        pathState = PathState.IDLE;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constance.createFollower(hardwareMap);


        buildPaths();
    }

    @Override
    public void start() {
        limelight.start();
        opModeTimer.resetTimer();
        setPathState(pathState);
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
        statePathUpdate();
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());
    }


}
