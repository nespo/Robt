from pyrplidar import PyRPlidar

try:
    lidar = PyRPlidar()
    
    # Connect to the Lidar device
    lidar.connect(port="/dev/ttyUSB0", baudrate=256000, timeout=3)
    # Linux: "/dev/ttyUSB0"
    # MacOS: "/dev/cu.SLAB_USBtoUART"
    # Windows: "COM5"
    
    print("PyRPlidar Info: Device connected")
    
    try:
        # Get device information
        info = lidar.get_info()
        print("Device Info:", info)
        
        # Get device health
        health = lidar.get_health()
        print("Device Health:", health)
        
        # Get current scan mode
        scan_modes = lidar.get_scan_modes()
        print("Scan Modes:")
        for scan_mode in scan_modes:
            print(scan_mode)
        
        # Get samplerate
        samplerate = lidar.get_samplerate()
        print("Samplerate:", samplerate)
    
    except IndexError as e:
        print("Error:", e)
    
    finally:
        # Disconnect from the Lidar device
        lidar.disconnect()
        print("PyRPlidar Info: Device disconnected")

except Exception as e:
    print("Connection Error:", e)
