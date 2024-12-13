package org.timecrafters.tacnet_management;

import android.content.Context;
import android.content.Intent;
import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.timecrafters.TimeCraftersConfigurationTool.library.tacnet.TACNETServerService;

@TeleOp(name = "Halt TACNET Service", group = "TACNET")
public class HaltTACNETService extends OpMode {
    @Override
    public void init() {
        Context appContext = hardwareMap.appContext;
        Intent tacnetIntent = new Intent(appContext, TACNETServerService.class);

        appContext.stopService(tacnetIntent);
    }

    @Override
    public void loop() {
        stop();
    }
}
