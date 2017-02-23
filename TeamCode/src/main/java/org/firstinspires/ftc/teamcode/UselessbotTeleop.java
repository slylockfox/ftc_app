package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.util.Range;

import static org.firstinspires.ftc.teamcode.HardwareUselessbot.ARM_FRONT;
import static org.firstinspires.ftc.teamcode.HardwareUselessbot.ARM_LEFT;
import static org.firstinspires.ftc.teamcode.HardwareUselessbot.ARM_LEFT_POWER;
import static org.firstinspires.ftc.teamcode.HardwareUselessbot.ARM_REAR;
import static org.firstinspires.ftc.teamcode.HardwareUselessbot.ARM_RIGHT;
import static org.firstinspires.ftc.teamcode.HardwareUselessbot.ARM_RIGHT_POWER;
import static org.firstinspires.ftc.teamcode.HardwareUselessbot.ARM_STOP_POWER;
import static org.firstinspires.ftc.teamcode.HardwareUselessbot.SHOULDER_LEVEL;

/**
 * This file provides basic Telop driving for a Pushbot robot.
 * The code is structured as an Iterative OpMode
 *
 * This OpMode uses the common Pushbot hardware class to define the devices on the robot.
 * All device access is managed through the HardwarePushbot class.
 *
 * This particular OpMode executes a basic Tank Drive Teleop for a PushBot
 * It raises and lowers the claw using the Gampad Y and A buttons respectively.
 * It also opens and closes the claws slowly using the left and right Bumper buttons.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Uselessbot Teleop", group="Uselessbot")
//@Disabled
public class UselessbotTeleop extends OpMode{

    /* Declare OpMode members. */
    HardwareUselessbot robot       = new HardwareUselessbot(); // use the class created to define a Pushbot's hardware
    // could also use HardwarePushbotMatrix class.
    double          shoulderPos  = SHOULDER_LEVEL ;                  // Servo mid position
    final double          shoulderInc = 0.008;
    //final double swivelInc = 0.01;
    double shoulderTime= 0.0;
    double swivelTime = 0.0;
    final double shoulderTimeDelay = 0.1;
    final double swivelTimeDelay = 0.1;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        updateTelemetry(telemetry);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        double left;
        double right;

        // Run wheels in tank mode (note: The joystick goes negative when pushed forwards, so negate it)
        left = -gamepad1.left_stick_y;
        right = -gamepad1.right_stick_y;
        robot.leftMotor.setPower(left);
        robot.rightMotor.setPower(right);

        // Use gamepad left & right Bumpers to swivel
        if (gamepad1.right_bumper)
            robot.swivelServo.setPower(ARM_RIGHT_POWER);
        else if (gamepad1.left_bumper)
            robot.swivelServo.setPower(ARM_LEFT_POWER);

        // use gamepad D buttons to swing to specific compass points
        else if (gamepad1.dpad_up)
            robot.swivelServo.setPower(robot.powerToPosition(ARM_FRONT));
        else if (gamepad1.dpad_right)
            robot.swivelServo.setPower(robot.powerToPosition(ARM_RIGHT));
        else if (gamepad1.dpad_down)
            robot.swivelServo.setPower(robot.powerToPosition(ARM_REAR));
        else if (gamepad1.dpad_left)
            robot.swivelServo.setPower(robot.powerToPosition(ARM_LEFT));

        // use A (green) and Y (yellow) to raise and lower shoulder
        else if (gamepad1.y && getRuntime() > shoulderTime + shoulderTimeDelay) {
            robot.shoulderServo.setPosition(shoulderPos += shoulderInc);
            shoulderTime = getRuntime();
        } else if (gamepad1.a && getRuntime() > shoulderTime + shoulderTimeDelay) {
            robot.shoulderServo.setPosition(shoulderPos -= shoulderInc);
            shoulderTime = getRuntime();
        }

        // no buttons; check for IR
        else if (robot.irSeekerH.signalDetected()) {
            robot.swivelServo.setPower(robot.IRMovementH());
            if (robot.irSeekerV.signalDetected() && getRuntime() > shoulderTime + shoulderTimeDelay) {
                robot.shoulderServo.setPosition(shoulderPos += robot.IRMovementV() * shoulderInc);
                shoulderTime = getRuntime();
            }
        }

        else { // no buttons or IR; stop swivel
            robot.swivelServo.setPower(ARM_STOP_POWER);
        }

        // Send telemetry message to signify robot running;
        telemetry.addData("AngleH",    robot.irSeekerH.getAngle());
        telemetry.addData("PercentRot", "percent: " + Double.toString(robot.armPosition()));
        telemetry.addData("left",  "%.2f", left);
        telemetry.addData("right", "%.2f", right);
        telemetry.addData("encoders",  "%7d - %7d",
                robot.leftMotor.getCurrentPosition(), robot.rightMotor.getCurrentPosition());
        updateTelemetry(telemetry);
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }



}
