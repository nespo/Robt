from pyrplidar import PyRPlidar

# Create an instance of PyRPlidar
lidar = PyRPlidar()

try:
    # Attempt to connect to the Lidar device
    lidar.connect(port="/dev/ttyUSB0", baudrate=256000, timeout=3)
    print("PyRPlidar Info: Device connected")

    try:
        # Fetch and print device information
        info = lidar.get_info()
        if info:
            print("Device Info:", info)
        else:
            print("Failed to get device info.")

        # Fetch and print device health
        health = lidar.get_health()
        if health:
            print("Device Health:", health)
        else:
            print("Failed to get device health.")

        # Fetch and print scan modes
        print("Getting scan modes...")
        scan_modes = lidar.get_scan_modes()
        if scan_modes:
            print("Scan Modes:")
            for scan_mode in scan_modes:
                print(scan_mode)
        else:
            print("No scan modes available.")

        # Fetch and print samplerate
        samplerate = lidar.get_samplerate()
        if samplerate:
            print("Samplerate:", samplerate)
        else:
            print("Failed to get samplerate.")

    except IndexError as e:
        # Log the exact location of the IndexError
        import traceback
        traceback.print_exc()
        print("Error: Index out of range", e)

    except Exception as e:
        # Handle other exceptions that may occur
        print("An error occurred:", e)

    finally:
        # Always disconnect from the Lidar device
        lidar.disconnect()
        print("PyRPlidar Info: Device disconnected")

except Exception as e:
    # Handle connection errors
    print("Connection Error:", e)
