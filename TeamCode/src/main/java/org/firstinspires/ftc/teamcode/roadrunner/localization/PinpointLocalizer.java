package org.firstinspires.ftc.teamcode.roadrunner.localization;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.UnnormalizedAngleUnit;

import java.util.Objects;

@Config
public final class PinpointLocalizer implements Localizer {
    public static class Params {
        // How far sideways (in inches) from the tracking point is the X (forward) odometry pod?
        // Left is positive, right is negative
        public double xPodOffset = -4.0;

        // How far forward (in inches) from the tracking point is the Y (strafe) odometry pod?
        // Forward is positive, back is negative
        public double yPodOffset = -4.0;

        public GoBildaPinpointDriver.GoBildaOdometryPods odometryPodType =
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_SWINGARM_POD;

        // Direction of the X (forward) encoder
        public GoBildaPinpointDriver.EncoderDirection xEncoderDirection =
                GoBildaPinpointDriver.EncoderDirection.FORWARD;
        // Direction of the Y (strafe) encoder
        public GoBildaPinpointDriver.EncoderDirection yEncoderDirection =
                GoBildaPinpointDriver.EncoderDirection.FORWARD;
    }

    public static Params PARAMS = new Params();

    public final GoBildaPinpointDriver driver;

    // Even though we don't use these internally, they still need to be set so that
    // they can be used by the TuningOpModes class.
    public final GoBildaPinpointDriver.EncoderDirection initialParDirection, initialPerpDirection;

    private Pose2d txWorldPinpoint;
    private Pose2d txPinpointRobot = new Pose2d(0, 0, 0);

    public PinpointLocalizer(HardwareMap hardwareMap, Pose2d initialPose) {
        driver = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        driver.setEncoderResolution(PARAMS.odometryPodType);
        driver.setOffsets(PARAMS.xPodOffset, PARAMS.yPodOffset, DistanceUnit.INCH);
        driver.setEncoderDirections(PARAMS.xEncoderDirection, PARAMS.yEncoderDirection);
        driver.resetPosAndIMU();

        initialParDirection = PARAMS.xEncoderDirection;
        initialPerpDirection = PARAMS.yEncoderDirection;

        txWorldPinpoint = initialPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        txWorldPinpoint = pose.times(txPinpointRobot.inverse());
    }

    @Override
    public Pose2d getPose() {
        return txWorldPinpoint.times(txPinpointRobot);
    }

    @Override
    public PoseVelocity2d update() {
        driver.update();
        if (Objects.requireNonNull(driver.getDeviceStatus()) == GoBildaPinpointDriver.DeviceStatus.READY) {
            txPinpointRobot = new Pose2d(driver.getPosX(DistanceUnit.INCH), driver.getPosY(DistanceUnit.INCH), driver.getHeading(UnnormalizedAngleUnit.RADIANS));
            Vector2d worldVelocity = new Vector2d(driver.getVelX(DistanceUnit.INCH), driver.getVelY(DistanceUnit.INCH));
            Vector2d robotVelocity = Rotation2d.fromDouble(-txPinpointRobot.heading.log()).times(worldVelocity);

            return new PoseVelocity2d(robotVelocity, driver.getHeadingVelocity(UnnormalizedAngleUnit.RADIANS));
        }
        return new PoseVelocity2d(new Vector2d(0, 0), 0);
    }
}
