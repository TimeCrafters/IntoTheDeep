package org.timecrafters.tacnet_management;

import android.content.Context;
import android.content.Intent;
import android.os.Build;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.timecrafters.TimeCraftersConfigurationTool.library.tacnet.TACNETServerService;

@TeleOp(name = "Start TACNET Service", group = "TACNET")
public class StartTACNETService extends OpMode {
    @Override
    public void init() {
        Context appContext = hardwareMap.appContext;
        Intent tacnetIntent = new Intent(appContext, TACNETServerService.class);

        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            appContext.startForegroundService(tacnetIntent);
        } else {
            appContext.startService(tacnetIntent);
        }
    }

    @Override
    public void loop() {
        stop();
    }
}
