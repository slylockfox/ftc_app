package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
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

@Autonomous(name="Test Vuforia", group = "Uselessbot")
@Disabled
public class UselessTestVuforia extends LinearOpMode {

    HardwareUselessPushbot robot   = new HardwareUselessPushbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

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
            for (VuforiaTrackable beac : beacons) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) beac.getListener()).getPose();
                if (pose!=null){
                    VectorF translation = pose.getTranslation();
                    telemetry.addData(beac.getName()+"-Translation", translation);
                    double degreesToTurn = Math.toDegrees(Math.atan2(translation.get(0), translation.get(1)));
                    telemetry.addData(beac.getName() + "-Degrees", degreesToTurn);
                }
            }
            telemetry.update();
        }

    }

}
