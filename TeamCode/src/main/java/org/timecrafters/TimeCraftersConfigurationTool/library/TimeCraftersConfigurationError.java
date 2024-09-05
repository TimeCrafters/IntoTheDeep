package org.timecrafters.TimeCraftersConfigurationTool.library;

public class TimeCraftersConfigurationError extends RuntimeException {
    public TimeCraftersConfigurationError() {
        super();
    }

    public TimeCraftersConfigurationError(String message) {
        super(message);
    }

    public TimeCraftersConfigurationError(String message, Throwable cause) {
        super(message, cause);
    }

    public TimeCraftersConfigurationError(Throwable cause) {
        super(cause);
    }

    public TimeCraftersConfigurationError(String message, Throwable cause, boolean enableSuppression, boolean writableStackTrace) {
        super(message, cause, enableSuppression, writableStackTrace);
    }
}
