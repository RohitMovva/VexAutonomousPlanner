import logging
import os

import yaml

logger = logging.getLogger(__name__)


class ConfigManager:
    """Handles loading, saving, and accessing configuration values."""

    def __init__(self, config_file_path="../config.yaml"):
        """Initialize the configuration manager.

        Args:
            config_file_path: Path to the configuration file (default: config.yaml)
        """
        self.config_file_path = config_file_path
        self.config = self._load_config()

    def _load_config(self):
        """Load configuration from YAML file.

        Returns:
            dict: Configuration values or default configuration if file doesn't exist
        """
        default_config = {
            "robot": {
                "width": 18.0,  # inches
                "length": 18.0,  # inches
                "track_width": 12.5,  # inches
                "visualization": False,  # robot visualization on/off
            },
            "motion": {
                "max_velocity": 4.0,  # ft/s
                "max_acceleration": 8.0,  # ft/s²
                "max_jerk": 16.0,  # ft/s³
            },
            "field": {
                "type": "Push Back Match",
            },
            "files": {
                "header_folder": "",  # Header file output folder
                "routes_folder": "",  # Routes folder
            },
        }

        if not os.path.exists(self.config_file_path):
            logger.info(
                f"Configuration file not found. Creating default at {self.config_file_path}"
            )
            self.config = default_config
            self._save_config()
            return default_config

        try:
            with open(self.config_file_path, "r") as file:
                config = yaml.safe_load(file)
                logger.info(f"Configuration loaded from {self.config_file_path}")

                # Ensure all default keys exist (in case config file is incomplete)
                if config is None:
                    config = default_config
                else:
                    # Make sure all sections and keys exist
                    for section, values in default_config.items():
                        if section not in config:
                            config[section] = values
                        else:
                            for key, value in values.items():
                                if key not in config[section]:
                                    config[section][key] = value

                return config
        except Exception as e:
            logger.error(f"Error loading configuration: {e}")
            return default_config

    def _save_config(self):
        """Save current configuration to YAML file."""
        try:
            with open(self.config_file_path, "w") as file:
                yaml.dump(self.config, file, default_flow_style=False)
            logger.info(f"Configuration saved to {self.config_file_path}")
            return True
        except Exception as e:
            logger.error(f"Error saving configuration: {e}")
            return False

    def get_value(self, section, key, default=None):
        """Get a configuration value.

        Args:
            section: Configuration section (e.g., 'robot', 'motion')
            key: Configuration key within the section
            default: Default value if the key doesn't exist

        Returns:
            The configuration value or default if not found
        """
        try:
            return self.config[section][key]
        except KeyError:
            return default

    def set_value(self, section, key, value):
        """Set a configuration value and save the configuration.

        Args:
            section: Configuration section (e.g., 'robot', 'motion')
            key: Configuration key within the section
            value: New value to set

        Returns:
            bool: True if successful, False otherwise
        """
        # Ensure section exists
        if section not in self.config:
            self.config[section] = {}

        # Set the value
        self.config[section][key] = value

        # Save the configuration
        return self._save_config()

    def get_all_config(self):
        """Get the entire configuration dictionary.

        Returns:
            dict: The complete configuration
        """
        return self.config
    
    def get_section(self, section, default=None):
        """Get a configuration section.

        Args:
            section: Configuration section (e.g., 'robot', 'motion')
            default: Default value if the section doesn't exist

        Returns:
            The configuration value or default if not found
        """
        try:
            return self.config[section]
        except KeyError:
            return default