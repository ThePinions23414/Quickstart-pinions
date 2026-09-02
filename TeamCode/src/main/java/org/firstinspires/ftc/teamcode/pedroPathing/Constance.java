package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constance {
    public static FollowerConstants followerConstants = new FollowerConstants()
//            .mass(7.6)
//            .forwardZeroPowerAcceleration(-25.08088842271777)
//            .lateralZeroPowerAcceleration(-55.71088307191639)
//            .translationalPIDFCoefficients(new PIDFCoefficients(0.4,0,0.005,0.2))
//            .headingPIDFCoefficients(new PIDFCoefficients(0.6, 0, 0.03, 0.023))
//            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.009, 0, 0.0001, 0.6, 0.025))
//            .centripetalScaling(0.0001)
            .mass(5.6)
            .forwardZeroPowerAcceleration(-27.366596724902912)
            .lateralZeroPowerAcceleration(-43.85411261809469)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.3,0,0.001,0.1))
            .headingPIDFCoefficients(new PIDFCoefficients(0.4, 0, 0.001, 0.02))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.008, 0, 0.0001, 0.6, 0.008))
            .centripetalScaling(0.00008)
            ;

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRight")
            .rightRearMotorName("backRight")
            .leftRearMotorName("backLeft")
            .leftFrontMotorName("frontLeft")
            .useBrakeModeInTeleOp(true)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
//            .xVelocity(87.15578959307332)
//            .yVelocity(70.69802207646408)
            .xVelocity(82.76324222594735)
            .yVelocity(70.08093502014643)
                ;

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(1.1)
            .strafePodX(-0.787)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            1,
            1);


    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                //.pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}
