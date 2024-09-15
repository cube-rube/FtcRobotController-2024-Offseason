package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.auto.Trajectory;
import org.firstinspires.ftc.teamcode.auto.TrajectoryBuilder;
import org.firstinspires.ftc.teamcode.auto.TrajectoryRunner;
import org.firstinspires.ftc.teamcode.file.FilesystemUtil;

import java.io.IOException;
import java.util.List;
import java.util.Objects;


public class Drive {
    private final DcMotorEx leftFrontDrive, leftBackDrive, rightFrontDrive, rightBackDrive;
    private final IMU imu;

    private final TwoWheelTrackingLocalizer localizer;

    private final TrajectoryRunner runner;


    public Drive(HardwareMap hardwareMap, Pose2d startPose) {
        // Motors
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        // Localizer
        localizer = new TwoWheelTrackingLocalizer(hardwareMap, this);
        localizer.setPoseEstimate(startPose);

        runner = new TrajectoryRunner();
    }

    public Drive(HardwareMap hardwareMap) {
        // Motors
        leftFrontDrive = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBackDrive = hardwareMap.get(DcMotorEx.class, "leftRear");
        rightFrontDrive = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBackDrive = hardwareMap.get(DcMotorEx.class, "rightRear");

        leftFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFrontDrive.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.DOWN, RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD));
        imu.initialize(parameters);
        imu.resetYaw();

        // Localizer
        localizer = new TwoWheelTrackingLocalizer(hardwareMap, this);
        localizer.setPoseEstimate(new Pose2d(0, 0, 0));

        runner = new TrajectoryRunner();
    }

    public void setMotorPowers(double lf, double lb, double rf, double rb) {
        leftFrontDrive.setPower(lf);
        leftBackDrive.setPower(lb);
        rightFrontDrive.setPower(rf);
        rightBackDrive.setPower(rb);
    }

    public void setMotorPowers(double[] motorPowers) {
        leftFrontDrive.setPower(motorPowers[0]);
        leftBackDrive.setPower(motorPowers[1]);
        rightFrontDrive.setPower(motorPowers[2]);
        rightBackDrive.setPower(motorPowers[3]);

    }

    public double[] setPowersByPose(Pose2d pose) {
        double axial = pose.getX();
        double lateral = -pose.getY();
        double yaw = pose.getHeading();

        double heading = localizer.getPoseEstimate().getHeading();

        double rotLateral = lateral * Math.cos(-heading) - axial * Math.sin(-heading);
        double rotAxial = lateral * Math.sin(-heading) + axial * Math.cos(-heading);

        rotLateral = rotLateral * 1.1;

        double leftFrontPower = rotAxial + rotLateral + yaw;
        double rightFrontPower = rotAxial - rotLateral - yaw;
        double leftBackPower = rotAxial - rotLateral + yaw;
        double rightBackPower = rotAxial + rotLateral - yaw;

        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        setMotorPowers(leftFrontPower,
                leftBackPower,
                rightFrontPower,
                rightBackPower);
        return new double[]{leftFrontPower, leftBackPower,
                rightFrontPower, rightBackPower};
    }

    public void updateLocalizer() {
        localizer.update();
    }

    public Pose2d getPoseEstimate() {
        return localizer.getPoseEstimate();
    }

    public double[] getMotorPowers() {
        return new double[]{leftFrontDrive.getPower(), leftBackDrive.getPower(),
                rightFrontDrive.getPower(), rightBackDrive.getPower()};
    }

    public List<Double> getWheelVelocities() {
        return localizer.getWheelVelocities();
    }

    public List<Double> getWheelPositions() {
        return localizer.getWheelPositions();
    }

    public double getParralelVelocity() {
        return Objects.requireNonNull(localizer.getWheelVelocities()).get(1);
    }

    public double getPerpendicularVelocity() {
        return Objects.requireNonNull(localizer.getWheelVelocities()).get(3);
    }

    public double getVelocityInTicks() {
        List<Double> wheelVelocities = localizer.getWheelVelocities();
        return Math.sqrt(wheelVelocities.get(1) * wheelVelocities.get(1) +
                wheelVelocities.get(3) * wheelVelocities.get(3));
    }

    public double getRawExternalHeading() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
    }

    public Double getExternalHeadingVelocity() {
        return (double) imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }

    public TrajectoryBuilder buildTrajectory(String file) throws IOException {
        return new TrajectoryBuilder(FilesystemUtil.loadFile(file));
    }

    public void followTrajectory(Trajectory trajectory) {
        localizer.setPoseEstimate(new Pose2d(trajectory.get(0).point0, Math.toRadians(0)));
        runner.followTrajectory(trajectory);
    }

    public void update() {
        Pose2d res = runner.update(getPoseEstimate());
        setPowersByPose(res);
    }
}
