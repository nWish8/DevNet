# sender.py

import network
import espnow # type: ignore
import utime
from machine import Pin

def run():
    # Sender logic
    sta = network.WLAN(network.STA_IF)
    sta.active(True)
    sta.disconnect()

    # Initialize ESP-NOW
    esp = espnow.ESPNow()
    esp.active(True)

    # Add the broadcast address to the peer list for discovering neighbors
    broadcast_mac = b'\xff\xff\xff\xff\xff\xff'
    esp.add_peer(broadcast_mac)

    # Function to discover neighbors by broadcasting
    def discover_neighbors():
        message = b'DISCOVER'
        print("Sending broadcast to discover neighbors...")

        # Send a broadcast message
        esp.send(broadcast_mac, message)

        # Listen for responses (timeout of 5 seconds)
        start_time = utime.ticks_ms()
        timeout = 5000  # Timeout in milliseconds
        neighbor_macs = []

        while utime.ticks_diff(utime.ticks_ms(), start_time) < timeout:
            sender, msg = esp.recv()  # Receive responses from peers
            if sender and msg == b'RESPONSE':  # If a neighbor responds with its MAC
                if sender not in neighbor_macs:
                    print(f"Received response from {sender}")
                    neighbor_macs.append(sender)  # Store the neighbor's MAC
                    try:
                        esp.add_peer(sender)  # Add the neighbor to the peer list
                        print(f"Successfully added peer {sender}")
                        return neighbor_macs # I had to add this in to stop the loop
                    except OSError as e:
                        print(f"Error adding peer {sender}: {e}")
        return neighbor_macs

    # Discover neighbors
    neighbors = discover_neighbors()

    # Print debug info for neighbors
    print(f"Neighbors discovered: {neighbors}")

    # Button pin setup and button press testing
    button_pin = Pin(23, Pin.IN, Pin.PULL_UP)
    last_button_state = button_pin.value()  # Initial button state

    # Directly check if we are getting valid button readings
    print(f"Initial button state: {last_button_state}")

    # If neighbors found, proceed
    if neighbors:
        print("neighbors found, entering button loop...")

        while True:
            current_button_state = button_pin.value()
            print(f"Current button state: {current_button_state}")  # Debugging button state

            # Check if button state has changed
            if current_button_state != last_button_state:
                # Send the appropriate command
                message = b'ledOn' if current_button_state == 0 else b'ledOff'
                print(f"Button state changed, sending: {message}")
                for neighbor_mac in neighbors:
                    print(f"Sending {message} to {neighbor_mac}")
                    esp.send(neighbor_mac, message)

                last_button_state = current_button_state  # Update button state

            utime.sleep(0.2)  # Sleep to avoid flooding the output
    else:
        print("No neighbors found. Exiting...")
