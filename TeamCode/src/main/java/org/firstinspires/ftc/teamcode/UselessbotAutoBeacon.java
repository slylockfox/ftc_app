package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import static org.firstinspires.ftc.teamcode.HardwareUselessbot.ARM_FRONT;
import static org.firstinspires.ftc.teamcode.HardwareUselessbot.ARM_REAR;
import static org.firstinspires.ftc.teamcode.HardwareUselessbot.ARM_RIGHT_POWER;
import static org.firstinspires.ftc.teamcode.HardwareUselessbot.ARM_STOP_POWER;
import static org.firstinspires.ftc.teamcode.HardwareUselessbot.PUSHER_EXTEND;
import static org.firstinspires.ftc.teamcode.HardwareUselessbot.PUSHER_RETRACT;

/**
 * Created by matt on 11/12/16.
 */

@Autonomous(name="Uselessbot Auto Beacon", group="Uselessbot")
@Disabled
public class UselessbotAutoBeacon extends LinearOpMode {

    HardwareUselessbot robot   = new HardwareUselessbot();
    boolean redAlliance = true;
    final double sweepStart = ARM_FRONT -1.0;
    final double maxDistanceToBeacon = 4.0;
    final double speed = 0.3;

    @Override
    public void runOpMode() throws InterruptedException {
        //motorLeft = hardwareMap.dcMotor.get("left_drive");
        //motorRight = hardwareMap.dcMotor.get("right_drive");
        robot.init(hardwareMap);

        while (!isStarted()) {  // basically waitforstart
            // allow changing alliance
            if (gamepad1.x) redAlliance = false;
            else if (gamepad1.b) redAlliance = true;
            telemetry.addData("Alliance", redAlliance ? "Red" : "Blue");
            telemetry.update();
            idle();
        }

        // start arm straight ahead
        for (double swivelPower = robot.powerToPosition(ARM_FRONT);
             swivelPower != ARM_STOP_POWER && opModeIsActive();
             swivelPower = robot.powerToPosition(ARM_FRONT)) {
            robot.swivelServo.setPower(swivelPower);
            idle();
        }
        robot.swivelServo.setPower(ARM_STOP_POWER);
        sleep(1000);

        // drive to beacon and get really close
        while (robot.rangeSensor.getDistance(DistanceUnit.CM) > maxDistanceToBeacon
                && opModeIsActive()) {
            telemetry.addData("cm", "%.2f cm", robot.rangeSensor.getDistance(DistanceUnit.CM));
            robot.leftMotor.setPower(speed);
            robot.rightMotor.setPower(speed);
        }
        robot.leftMotor.setPower(0.0);
        robot.rightMotor.setPower(0.0);

        // start arm on left
        for (double swivelPower = robot.powerToPosition(sweepStart);
             swivelPower != ARM_STOP_POWER && opModeIsActive();
             swivelPower = robot.powerToPosition(sweepStart)) {
            robot.swivelServo.setPower(swivelPower);
            idle();
        }

        // swing left to right, pushing button if color matches alliance
        while (robot.armPosition() < ARM_FRONT + 1.0 && opModeIsActive()) {

            // move arm
            robot.swivelServo.setPower(ARM_RIGHT_POWER);
            sleep(40);
            robot.swivelServo.setPower(ARM_STOP_POWER);

            // push button if color is right
            if (   (redAlliance && robot.colorSensor.red() > robot.colorSensor.blue())
                || (!redAlliance && robot.colorSensor.blue() > robot.colorSensor.red()) )
            robot.buttonPusher.setPosition(PUSHER_EXTEND);
            sleep(500);
            robot.buttonPusher.setPosition(PUSHER_RETRACT);
            sleep(500);

            idle();
        }

    }

}
