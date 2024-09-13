# receiver.py

import network
import espnow # type: ignore
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
    def reset_and_add_peer(sender):
        try:
            esp.del_peer(sender)  # Delete the peer if it exists
        except OSError:
            pass  # Ignore if peer doesn't exist

        try:
            esp.add_peer(sender)  # Add the peer
            print(f"Peer {sender} added successfully.")
        except OSError as e:
            print(f"Error adding peer {sender}: {e}")

    # Function to handle DISCOVER messages
    def handle_discover(sender):
        reset_and_add_peer(sender)  # Ensure peer is reset and added before responding
        print(f"Received DISCOVER from {sender}, sending response.")
        try:
            esp.send(sender, b'RESPONSE')  # Send the response after adding peer
        except OSError as e:
            print(f"Error sending response to {sender}: {e}")

    # Main loop to listen for messages
    while True:
        sender, msg = esp.recv()  # Listen for a message from any peer (sender)
        if msg:
            print(f"Received message: {msg} from {sender}")
            if msg == b'DISCOVER':  # If the message is a DISCOVER request
                handle_discover(sender)  # Reset peer and send response
            elif msg == b'ledOn':  # If the sender asks to turn on the LED
                print("Turning on LED")
                led_pin.on()
            elif msg == b'ledOff':  # If the sender asks to turn off the LED
                print("Turning off LED")
                led_pin.off()
            else:
                print("Unknown message received!")
