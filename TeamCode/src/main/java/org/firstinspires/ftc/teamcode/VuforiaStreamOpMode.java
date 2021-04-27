package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/*
 * This sample demonstrates how to stream frames from Vuforia to the dashboard. Make sure to fill in
 * your Vuforia key below and select the 'Camera' preset on top right of the dashboard. This sample
 * also works for UVCs with slight adjustments.
 */
@Autonomous
public class VuforiaStreamOpMode extends LinearOpMode {

    // TODO: fill in
    public static final String VUFORIA_LICENSE_KEY = "AUW3eZ//////AAABmfdfbFK2c0SCqCcABud+QtoCs8D52Dju3cJDcuHPVETOY1of+fkgcgsIRLZXjNtwfGVKS3JxMmiBw3v8828JM1YzW4hGSk7w6XivhUAg88AjpUcl6QdsAcmdATiddprbrTGc6g6OBmhg8n0Te/w2bp9al+lZlR9O20Gbm7PT2j32Lo/luy/OSbgqzgiq/EzHYlt3c4oi4WE5dtEOBAmUcJf+Gx4NqPkqcZhuZtGu0O3HX+Zsot9er30nwAdFTotIG0SyzXCFlOm41xe773GrPnnro+XVO1evfM2VuNYqaknRBUkfEWZHw5NWBfEnVIqRvtAwUuCd6qj2LTGaQLpr/ZKtGVEMBbLJ4Wu17tlaRvXz";

    @Override
    public void runOpMode() throws InterruptedException {
        // gives Vuforia more time to exit before the watchdog notices
        msStuckDetectStop = 2500;

        VuforiaLocalizer.Parameters vuforiaParams = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
        vuforiaParams.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        vuforiaParams.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VuforiaLocalizer vuforia = ClassFactory.getInstance().createVuforia(vuforiaParams);

        FtcDashboard.getInstance().startCameraStream(vuforia, 0);

        waitForStart();

        while (opModeIsActive());
    }
}