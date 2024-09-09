# receiver.py

import network
import espnow
import machine

def run():
    # Receiver logic
    sta = network.WLAN(network.STA_IF)
    sta.active(True)
    sta.disconnect()

    # Initialize ESP-NOW
    esp = espnow.ESPNow()
    esp.active(True)

    led_pin = machine.Pin(22, machine.Pin.OUT)

    # Function to clear the peer list and re-add
    def reset_and_add_peer(peer_mac):
        try:
            esp.del_peer(peer_mac)  # Delete the peer if it exists
        except OSError:
            pass  # Ignore if peer doesn't exist

        try:
            esp.add_peer(peer_mac)  # Add the peer
            print(f"Peer {peer_mac} added successfully.")
        except OSError as e:
            print(f"Error adding peer {peer_mac}: {e}")

    # Function to handle DISCOVER messages
    def handle_discover(peer_mac):
        reset_and_add_peer(peer_mac)  # Ensure peer is reset and added before responding
        print(f"Received DISCOVER from {peer_mac}, sending response.")
        try:
            esp.send(peer_mac, b'RESPONSE')  # Send the response after adding peer
        except OSError as e:
            print(f"Error sending response to {peer_mac}: {e}")

    # Main loop to listen for messages
    while True:
        peer_mac, msg = esp.recv()  # Listen for a message from any peer (sender)
        if msg:
            print(f"Received message: {msg} from {peer_mac}")
            if msg == b'DISCOVER':  # If the message is a DISCOVER request
                handle_discover(peer_mac)  # Reset peer and send response
            elif msg == b'ledOn':  # If the sender asks to turn on the LED
                print("Turning on LED")
                led_pin.on()
            elif msg == b'ledOff':  # If the sender asks to turn off the LED
                print("Turning off LED")
                led_pin.off()
            else:
                print("Unknown message received!")
