#!/usr/bin/env python3
"""
Example usage of the Nextwaves RFID SDK

This script demonstrates how to use the Nextwaves RFID SDK for basic operations.
"""

import logging
import time
from nrn import NRNReader, create_reader, setup_logging

def main():
    """Main example function."""
    
    # Setup logging
    logger = setup_logging(level=logging.INFO)
    logger.info("Starting Nextwaves RFID SDK Example")
    
    # Create reader instance using factory function (recommended)
    reader = create_reader(
        port="/dev/ttyUSB0",  # Change this to your actual port
        baudrate=115200,
        log_level=logging.INFO
    )
    
    try:
        # Get SDK information
        sdk_info = NRNReader.get_sdk_info()
        logger.info(f"Using {sdk_info['name']} v{sdk_info['version']}")
        
        # Get reader configuration
        reader_info = reader.get_reader_info()
        logger.info(f"Reader configured: {reader_info}")
        
        # Connect to reader
        reader.open()
        
        # Get reader information
        info = reader.Query_Reader_Information()
        if info:
            logger.info(f"Reader Serial: {info.get('serial_number', 'Unknown')}")
            logger.info(f"Power-on time: {info.get('power_on_time_sec', 0)} seconds")
        
        # Query RFID capabilities
        capabilities = reader.query_rfid_ability()
        if capabilities:
            logger.info(f"Max power: {capabilities.get('max_power_dbm', 'Unknown')} dBm")
            logger.info(f"Min power: {capabilities.get('min_power_dbm', 'Unknown')} dBm")
            logger.info(f"Antenna count: {capabilities.get('antenna_count', 'Unknown')}")
        
        # Example: Start inventory with callback
        tags_found = []
        
        def on_tag_callback(tag):
            """Callback function for when tags are found."""
            tag_info = f"EPC: {tag.get('epc', 'Unknown')}, "
            tag_info += f"RSSI: {tag.get('rssi', 'Unknown')}, "
            tag_info += f"Antenna: {tag.get('antenna_id', 'Unknown')}"
            logger.info(f"Tag found: {tag_info}")
            tags_found.append(tag)
        
        logger.info("Starting inventory for 10 seconds...")
        reader.start_inventory_with_mode(antenna_mask=[1], callback=on_tag_callback)
        
        # Run inventory for 10 seconds
        time.sleep(10)
        
        # Stop inventory
        reader.stop_inventory()
        logger.info(f"Inventory completed. Found {len(tags_found)} unique tags.")
        
        # Example: Configure reader power
        power_settings = {1: 30}  # Set antenna 1 to 30dBm
        if reader.configure_reader_power(power_settings):
            logger.info("Power configuration successful")
        else:
            logger.warning("Power configuration failed")
        
        # Example: Query current power settings
        current_power = reader.query_reader_power()
        if current_power:
            logger.info(f"Current power settings: {current_power}")
        
    except Exception as e:
        logger.error(f"Error during operation: {e}")
    
    finally:
        # Always close the connection
        reader.close()
        logger.info("Example completed")

if __name__ == "__main__":
    main()
