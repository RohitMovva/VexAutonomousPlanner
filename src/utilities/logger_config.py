import logging
import os
import sys
from datetime import datetime
from enum import Enum
from logging.handlers import RotatingFileHandler


class LogMode(Enum):
    NONE = "none"  # No logging
    FILE_ONLY = "file"  # Log to file only
    CONSOLE_ONLY = "console"  # Log to console only
    ALL = "all"  # Log to both file and console


def setup_global_logger(level=logging.INFO, mode=LogMode.ALL):
    """
    Configure a global logger with flexible output options

    Args:
        level: Logging level (e.g., logging.INFO, logging.DEBUG)
        mode: LogMode enum determining where logs should be output
    """

    # If logging is disabled, set up a null handler
    if mode == LogMode.NONE:
        logging.getLogger().addHandler(logging.NullHandler())
        return

    # Get the root logger
    logger = logging.getLogger()
    logger.setLevel(level)

    # Clear any existing handlers
    logger.handlers = []

    # Create formatter
    formatter = logging.Formatter(
        "%(asctime)s - %(name)s - %(levelname)s - %(message)s"
    )

    # Set up console logging if requested
    if mode in [LogMode.CONSOLE_ONLY, LogMode.ALL]:
        console_handler = logging.StreamHandler(sys.stdout)
        console_handler.setFormatter(formatter)
        logger.addHandler(console_handler)

    # Set up file logging if requested
    if mode in [LogMode.FILE_ONLY, LogMode.ALL]:
        # Create logs directory
        logs_dir = "logs"
        if not os.path.exists(logs_dir):
            os.makedirs(logs_dir)

        # Create directory for today's date
        today = datetime.now().strftime("%Y%m%d")
        daily_dir = os.path.join(logs_dir, today)
        if not os.path.exists(daily_dir):
            os.makedirs(daily_dir)

        # Create timestamp for log file name
        timestamp = datetime.now().strftime("%H%M%S")
        log_file = os.path.join(daily_dir, f"app_{timestamp}.log")

        # File handler
        file_handler = RotatingFileHandler(
            log_file,
            maxBytes=1024 * 1024,  # 1MB
            backupCount=3,
        )
        file_handler.setFormatter(formatter)
        logger.addHandler(file_handler)

        # Log the file location (only if console logging is enabled)
        if mode == LogMode.ALL:
            logger.info(f"Logging started - Log file: {log_file}")


def set_log_level(level):
    """Dynamically change the log level"""
    logging.getLogger().setLevel(level)
