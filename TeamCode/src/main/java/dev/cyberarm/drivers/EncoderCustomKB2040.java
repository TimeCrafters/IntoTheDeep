package dev.cyberarm.drivers;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.annotations.DeviceProperties;
import com.qualcomm.robotcore.hardware.configuration.annotations.I2cDeviceType;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

@I2cDeviceType()
@DeviceProperties(name = "Encoder Custom KB2040", description = "Non-competition legal i2c encoder", xmlTag = "ENCODER_CUSTOM_KB2040")
public class EncoderCustomKB2040 extends I2cDeviceSynchDevice<I2cDeviceSynchSimple> {
    enum Register {
        REPORT_POSITION,
        RESET_POSITION
    }

    private final static I2cAddr ADDRESS_I2C_DEFAULT = I2cAddr.create7bit(0x16);
    private int position = -1;
    public EncoderCustomKB2040(I2cDeviceSynchSimple i2cDeviceSynchSimple, boolean deviceClientIsOwned) {
        super(i2cDeviceSynchSimple, deviceClientIsOwned);

        this.deviceClient.setI2cAddress(ADDRESS_I2C_DEFAULT);
        super.registerArmingStateCallback(false);
        engage();
    }

    @Override
    protected synchronized boolean doInitialize() {
        return true;
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Encoder Custom KB2040";
    }

    public int getCurrentPosition() {
//        deviceClient.write8(Register.READ.ordinal());

        // Read int32_t
        byte[] buffer = deviceClient.read(Register.REPORT_POSITION.ordinal(), 4);

        // Reconstruct int32_t from 4 int8_t
        int newPos = ByteBuffer.wrap(buffer).order(ByteOrder.BIG_ENDIAN).getInt();

        position = newPos;

        return position;
    }

    public int getLastPosition() {
        return position;
    }

    public void reset() {
        deviceClient.write8(Register.RESET_POSITION.ordinal(), I2cWaitControl.WRITTEN);
    }
}
