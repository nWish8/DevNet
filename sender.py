# sender.py

import network
import espnow
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

    # Add the broadcast address to the peer list for discovering receivers
    broadcast_mac = b'\xff\xff\xff\xff\xff\xff'
    esp.add_peer(broadcast_mac)

    # Function to discover receivers by broadcasting
    def discover_receivers():
        message = b'DISCOVER'
        print("Sending broadcast to discover receivers...")

        # Send a broadcast message
        esp.send(broadcast_mac, message)

        # Listen for responses (timeout of 5 seconds)
        start_time = utime.ticks_ms()
        timeout = 5000  # Timeout in milliseconds
        receiver_macs = []

        while utime.ticks_diff(utime.ticks_ms(), start_time) < timeout:
            peer_mac, msg = esp.recv()  # Receive responses from peers
            if peer_mac and msg == b'RESPONSE':  # If a receiver responds with its MAC
                if peer_mac not in receiver_macs:
                    print(f"Received response from {peer_mac}")
                    receiver_macs.append(peer_mac)  # Store the receiver's MAC
                    try:
                        esp.add_peer(peer_mac)  # Add the receiver to the peer list
                        print(f"Successfully added peer {peer_mac}")
                        return receiver_macs # I had to add this in to stop the loop
                    except OSError as e:
                        print(f"Error adding peer {peer_mac}: {e}")
        return receiver_macs

    # Discover receivers
    receivers = discover_receivers()

    # Print debug info for receivers
    print(f"Receivers discovered: {receivers}")

    # Button pin setup and button press testing
    button_pin = Pin(23, Pin.IN, Pin.PULL_UP)
    last_button_state = button_pin.value()  # Initial button state

    # Directly check if we are getting valid button readings
    print(f"Initial button state: {last_button_state}")

    # If receivers found, proceed
    if receivers:
        print("Receivers found, entering button loop...")

        while True:
            current_button_state = button_pin.value()
            print(f"Current button state: {current_button_state}")  # Debugging button state

            # Check if button state has changed
            if current_button_state != last_button_state:
                # Send the appropriate command
                message = b'ledOn' if current_button_state == 0 else b'ledOff'
                print(f"Button state changed, sending: {message}")
                for receiver_mac in receivers:
                    print(f"Sending {message} to {receiver_mac}")
                    esp.send(receiver_mac, message)

                last_button_state = current_button_state  # Update button state

            utime.sleep(0.2)  # Sleep to avoid flooding the output
    else:
        print("No receivers found. Exiting...")
