package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.HINT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

/**
 * Created by matt on 10/10/16.
 */

@Autonomous(name="Vuforia Swerve to Center", group = "Test")
public class VuforiaLinearOp extends LinearOpMode {

    MattHardwarePushbotReverse     robot   = new MattHardwarePushbotReverse();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();
        robot.leftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        idle();
        robot.leftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0",  "Starting at %7d :%7d",
                robot.leftMotor.getCurrentPosition(),
                robot.rightMotor.getCurrentPosition());
        telemetry.update();

        VuforiaLocalizer.Parameters params = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        params.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        params.vuforiaLicenseKey = "ARda7ET/////AAAAGRKVBrspsk3lhF5JeNbnxGoTXHlCxkXqkmsmgZr7/9G89bz9B9s3pAZj4WcSfAWfuDSgKQ72SgBkTODn75Vektd1v2dqVUC7qCIBX3iVBDm3NxxJbDc8UGRdmWAkjDO+9fxktZiKIdgGKkmLq2OQiYWNKf5zZYBtnJhjH+Rh+Q4gVFb5R+AL5ujKfKV4AQL+nbQ8ow9rRF5reWgTBleibhrYaecm9SEjF209yjAjGWToxd5LlhNEzH3nG/yWBl3kG6ArLNpy/E8WqfvU3UnFpFQ59tOtuE8Y53QQoKCazAiNASN3WLDH5hCMXxh+6tTmHnLpLhKeN+05DgEsD5JnssCdNaea/LWOcYxyqtdqzjQB";
        params.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

        VuforiaLocalizer vuforia = ClassFactory.createVuforiaLocalizer(params);
        Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);

        VuforiaTrackables beacons = vuforia.loadTrackablesFromAsset("FTC_2016-17");
        beacons.get(0).setName("Wheels");
        beacons.get(1).setName("Tools");
        beacons.get(2).setName("Legos");
        beacons.get(3).setName("Gears");

        waitForStart();

        beacons.activate();

        while (opModeIsActive()){
            VuforiaTrackable beac = beacons.get(0); // wheels
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
            if (pose!=null){

                VectorF translation = pose.getTranslation();
                telemetry.addData(beac.getName()+"-Translation", translation);
                double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(0), translation.get(1)));
                double distance = Math.abs(translation.get(2));
                telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);
                telemetry.addData(beac.getName() + "-Distance", distance);

                // orient robot to center image, using semi proportional drive
                // double speed = Math.abs(degreesToTurn) > 90 ? 0.2 : 0.05;
                double leftSpeed = 0, rightSpeed = 0;
                if (distance > 300) {
                    if (Math.abs(degreesToTurn) < 20) { // drive straight
                        leftSpeed = 0.1;
                        rightSpeed = 0.1;
                    } else if (degreesToTurn > 0) { // veer right
                        leftSpeed = 0.15;
                        rightSpeed = 0.1;
                    } else if (degreesToTurn < 0) { // veer left
                        leftSpeed = 0.1;
                        rightSpeed = 0.15;
                    }
                }
                telemetry.addData("Left Speed", leftSpeed);
                telemetry.addData("Right Speed", rightSpeed);
                robot.leftMotor.setPower(leftSpeed);
                robot.rightMotor.setPower(rightSpeed);
            }
        telemetry.update();
        }

        // Stop all motion;
        robot.leftMotor.setPower(0);
        robot.rightMotor.setPower(0);

    }

}
