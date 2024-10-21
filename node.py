import config  # Import the config module
import network
import espnow  # type: ignore
import utime
import random
import json
from machine import SoftI2C, Pin, UART, RTC, ADC
import BME280
import utelnetserver
import struct

TX_POWER = 2  # Maximum TX power in dBm

GATEWAY_MAC = 'e8:31:cd:70:f1:6c'  # Gateway MAC address
INTERVAL = 10  # Interval for sending data (in seconds)

REPLY_TIMEOUT = 3  # Should be less than INTERVAL
WAIT_TIME = 3  # Time in seconds to wait for downstream dreps

DELAY = 1  # Delay for reading GPS data
WEIGHT = 0.5  # Weight for calculating overhead (battery level and RSSI)

# Define message type constants
DREQ_MSG = 1  # Message type for 'dreq'
DREP_MSG = 2  # Message type for 'drep'

# Define struct format for packed data
# Format: node_id (6s), timestamp (I), latitude (f), longitude (f),
# temperature (h), humidity (B), pressure (h), battery (h), rssi (b), hop_count (B)
STRUCT_FORMAT = '>6sIffhBhhbB'
STRUCT_SIZE = struct.calcsize(STRUCT_FORMAT)  # Should be 25 bytes per node

###########################################################
def run():
    '''
    Main function to run the node.
    '''
    node = MyNode()
    node.run()  # Start the event loop

###########################################################
class MyNode:
    def __init__(self):
        '''
        Initialize the node.
        '''

        ssid = config.WIFI_SSID
        password = config.WIFI_PASSWORD

        # Initialize WLAN
        self.sta = network.WLAN(network.STA_IF)
        self.sta.active(True)
        self.sta.connect(ssid, password)

        # Set self.node_id to the MAC address in readable format
        self.node_id = ':'.join(f'{b:02x}' for b in self.sta.config('mac'))

        if self.node_id == GATEWAY_MAC:
            while not self.sta.isconnected():
                utime.sleep(0.1)
            print('Connected, IP:', self.sta.ifconfig()[0])

        self.sta.config(txpower=TX_POWER)  # Set the TX power
        self.log(network.WLAN(network.STA_IF).config("txpower"))  # Get the TX power

        # Initialize ESP-NOW
        self.esp = espnow.ESPNow()
        self.esp.active(True)

        # Start the Telnet server for remote terminal access
        utelnetserver.start()

        # Add the broadcast address to the peer list for discovering receivers
        self.broadcast_mac = b'\xff\xff\xff\xff\xff\xff'
        self.esp.add_peer(self.broadcast_mac)

        # Initialize I2C for BME280 sensor
        i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=100000)

        # Initialize UART for GPS
        self.gps_uart = UART(1, baudrate=9600, tx=16, rx=17)  # Use UART1 for GPS (TX=GPIO17, RX=GPIO16)
        # Initialize PPS pin
        self.pps_pin = Pin(33, Pin.IN)  # GPIO33 connected to PPS signal
        # Attach an interrupt to the PPS pin
        #self.pps_pin.irq(trigger=Pin.IRQ_RISING, handler=self.handle_pps_pulse)

        # Initialize BME280 sensor
        try:
            self.bme = BME280.BME280(i2c=i2c)  # Adjust address if necessary
            self.log("BME280 sensor initialized successfully.")
        except OSError as e:
            self.log(f"OSError occurred during BME280 sensor initialization: {str(e)}")
            self.bme = None

        # Initialize ADC for battery level reading
        try:
            self.adc = ADC(Pin(34))  # Assuming battery voltage connected to GPIO34
            self.adc.atten(ADC.ATTN_11DB)  # Set attenuation for full range
            self.log("ADC initialized for battery level monitoring.")
        except Exception as e:
            self.log(f"Error initializing ADC: {str(e)}")

        # Initialize GSM module only on the gateway node
        if self.node_id == GATEWAY_MAC:
            try:
                # Initialize UART for GSM
                self.gsm_uart = UART(2, baudrate=115200, tx=25, rx=26)  # Use UART2 for GSM (TX=GPIO25, RX=GPIO26)

                # Initialize pins for GSM control
                self.gsm_pwr = Pin(27, Pin.OUT)
                self.gsm_rst = Pin(14, Pin.OUT)

                # Turn on the GSM module
                self.gsm_pwr.on()
                utime.sleep(1)  # Wait for the module to power up

                # Reset the GSM module
                self.gsm_rst.off()
                utime.sleep(0.1)
                self.gsm_rst.on()

                self.log("GSM module initialized successfully.")
            except Exception as e:
                self.log(f"Error initializing GSM module: {str(e)}")
                self.gsm_uart = None  # Set to None if initialization fails
        else:
            self.gsm_uart = None  # GSM module not present on other nodes

        self.update_gps()  # Fetch GPS data for the first time

        self.start_seq = True
        self.last_request_time = 0  # Store the time when the request was sent
        self.tx_counter = 0
        self.neighbor_table = {}  # Dictionary to store neighbors, their overheads, and paths
        self.hop_count = 0  # Hop count to the gateway
        self.battery_level = 0.0
        self.rssi = 0.0
        self.apparent_battery = 0
        self.apparent_rssi = 0.0
        self.overhead = 0.0
        self.path = GATEWAY_MAC  # Path to the gateway
        self.clock_offset = 0  # Clock offset for synchronization
        self.start_flag = False
        self.data_cache = {}
        self.latency = 0
        self.position = [0.0, 0.0]  # Initialize position

    def run(self):
        '''Main loop for node'''

        while True:
            if self.node_id == GATEWAY_MAC and self.start_seq:
                self.log("STARTING...")
                utime.sleep(1)
                self.send_dreq()
                self.last_request_time = utime.time()
                self.latency = utime.ticks_ms()
                self.start_seq = False

            # Check if time since request exceeds REPLY_TIMEOUT seconds and no reply received
            if self.node_id == GATEWAY_MAC and not self.start_seq:
                current_time = utime.time()
                if current_time - self.last_request_time > REPLY_TIMEOUT + 5:
                    self.log(f"Reply Timeout: {REPLY_TIMEOUT + 5} seconds.")
                    self.start_flag = True
                    #self.start_seq = True

            end_time = utime.time() + WAIT_TIME
            while utime.time() < end_time:
                try:
                    sender, data = self.esp.recv(0)  # Non-blocking receive
                    if sender and data:
                        sender_mac = ':'.join(f'{b:02x}' for b in sender)
                        self.on_receive(sender_mac, data)
                except OSError:
                    pass
                utime.sleep(0.1)  # Small delay to prevent a tight loop

            self.start_reply()

            utime.sleep(0.1)


    def send_dreq(self):
        '''
        Send a Data Request (dreq) message to the broadcast address.
        '''
        data = {
            'm': 'dreq',
            'tx': self.tx_counter,
            'b': self.apparent_battery,
            'r': self.apparent_rssi,
            'o': self.overhead,
            'p': self.path,
            'c': self.now
        }
        json_data = json.dumps(data).encode('utf-8')  # Encode data to JSON

        # Prepend the message type byte
        msg_type = bytes([DREQ_MSG])  # 0x01 for 'dreq' messages
        full_data = msg_type + json_data

        utime.sleep(self.rand_delay())  # Random delay to avoid collisions

        try:
            self.esp.send(self.broadcast_mac, full_data)
            self.log(f"SENT: dreq from {self.node_id}")
        except Exception as e:
            self.log(f"Error sending dreq: {str(e)}")

    def send_dreply(self):
        '''
        Send a Data Reply (drep) message to the next node in the path.
        '''
        # Prepare packed data
        packed_data = b''
        for node_id_str, node_data in self.data_cache.items():
            node_id_bytes = bytes(int(b, 16) for b in node_id_str.split(':'))

            # Use current system timestamp
            timestamp = int(utime.time())

            # Pack data
            packed_node_data = struct.pack(
                STRUCT_FORMAT,
                node_id_bytes,
                timestamp,
                float(node_data['P'][0]),   # Latitude
                float(node_data['P'][1]),   # Longitude
                int(node_data['t'] * 10),   # Temperature * 10
                int(node_data['h']),        # Humidity
                int(node_data['p']),        # Pressure
                int(node_data['b'] * 10),   # Battery Level * 10
                int(node_data['r']),        # RSSI (signed char)
                int(node_data['H'])         # Hop Count
            )
            packed_data += packed_node_data

        mac_bytes = bytes(int(b, 16) for b in self.path.split(':'))

        # Prepend the message type
        msg_type = bytes([DREP_MSG])  # 0x02 for 'drep' messages
        full_data = msg_type + packed_data

        self.log(f"Data length: {len(full_data)} bytes")

        utime.sleep(self.rand_delay())  # Random delay to avoid collisions

        try:
            self.esp.send(mac_bytes, full_data)
            self.log(f"SENT: drep to {self.path}")
        except Exception as e:
            self.log(f"Error sending drep: {str(e)}")

    def on_receive(self, sender_mac, data):
        '''
        Handle received messages and act based on the message type.
        '''
        # Determine the message type from the first byte
        msg_type = data[0]
        data = data[1:]  # Remove the message type byte

        if msg_type == DREQ_MSG:
            # Handle 'dreq' messages, which are in JSON
            try:
                data = json.loads(data.decode('utf-8'))
                msg = data['m']
                self.log(f"RECEIVED: {msg} from {sender_mac}")
                # Existing code for handling 'dreq' messages
                self.handle_dreq(sender_mac, data)
            except Exception as e:
                self.log(f"Error processing dreq: {str(e)}")
        elif msg_type == DREP_MSG:
            msg = 'drep'
            self.log(f"RECEIVED: {msg} from {sender_mac}")
            # Unpack the binary data
            offset = 0
            data_length = len(data)
            while offset < data_length:
                try:
                    unpacked_data = struct.unpack_from(STRUCT_FORMAT, data, offset)
                    offset += STRUCT_SIZE

                    node_id_bytes = unpacked_data[0]
                    node_id_str = ':'.join('{:02x}'.format(b) for b in node_id_bytes)

                    # Reconstruct time from timestamp
                    timestamp = unpacked_data[1]
                    time_struct = utime.localtime(timestamp)
                    time_str = "{:02}:{:02}:{:02}".format(time_struct[3], time_struct[4], time_struct[5])

                    node_data = {
                        'c': time_str,
                        'i': node_id_str,
                        'P': [unpacked_data[2], unpacked_data[3]],
                        't': unpacked_data[4] / 10.0,
                        'h': unpacked_data[5],
                        'p': unpacked_data[6],
                        'b': unpacked_data[7] / 10.0,
                        'r': unpacked_data[8],
                        'H': unpacked_data[9]
                    }

                    # Increment hop count
                    node_data['H'] += 1

                    # Update the data cache
                    self.data_cache[node_id_str] = node_data

                except Exception as e:
                    self.log(f"Error unpacking drep data: {str(e)}")
                    break  # Exit the loop on error

            # Handle the drep message
            self.handle_drep(sender_mac)
        else:
            self.log(f"Unknown message type {msg_type} from {sender_mac}")

    def handle_dreq(self, sender_mac, data):
        '''
        Handle received dreq messages and act accordingly.
        '''
        self.is_new_transaction(data)  # If new transaction, reset the node

        if not self.neighbor_table:  # First DREQ message received
            self.update_neighbor_table(sender_mac, data)  # Update the neighbor table
            if self.node_id != GATEWAY_MAC:
                self.path = sender_mac  # Update self.path
                self.overhead = self.calculate_overhead(sender_mac, data) + data['o']  # Calculate node overhead
                # utime.sleep(self.rand_delay())
                self.send_dreq()  # Forward the DREQ message
            self.start_flag = True  # Set the start flag to True

        else:  # Node has received a DREQ message before
            if sender_mac not in self.neighbor_table or data['o'] < self.neighbor_table[sender_mac]['overhead']:
                self.update_neighbor_table(sender_mac, data)  # Update the neighbor table

                # Check if the new received overhead is less than the current lowest overhead neighbor
                lowest_neighbor = self.lowest_overhead_neighbor()
                if data['o'] < self.neighbor_table[lowest_neighbor]['overhead']:
                    if self.node_id != GATEWAY_MAC:
                        self.path = sender_mac  # Update self.path
                        self.overhead = self.calculate_overhead(sender_mac, data) + data['o']  # Update node overhead
                        # utime.sleep(self.rand_delay())
                        self.send_dreq()  # Forward the DREQ message
                    self.start_flag = True  # Set the start flag to True

    def handle_drep(self, sender_mac):
        '''
        Handle received drep messages and act accordingly.
        '''
        self.neighbor_table.setdefault(sender_mac, {})['rx'] = 1  # Update neighbor packets received

        # Only add own data if not already added
        if self.node_id not in self.data_cache:
            self.data_cache_update()  # Add own data to the data cache

        # Log the neighbor table entries where 'path' == self.node_id
        branches = {key: neighbor for key, neighbor in self.neighbor_table.items() if neighbor.get('path') == self.node_id}

        # Check if all branches have 'rx' == 1
        if all(neighbor.get('rx', 0) == 1 for neighbor in branches.values()):
            if self.node_id != GATEWAY_MAC:  # If this node is not the gateway, forward the data packet
                # utime.sleep(self.rand_delay())  # Wait for a random delay
                self.send_dreply()  # Send the data packet to the next node
            else:  # All data has reached the gateway
                self.log_data()  # Log the data
                self.log(f"DATA:\n" + self.init_data_cache())
                self.log(f"COMPLETE")
                self.tx_counter += 1
                self.start_seq = True
                self.neighbor_table = {}  # Reset the Gateway neighbor table
                self.data_cache = {}  # Reset the data cache
                utime.sleep(INTERVAL)
            self.start_flag = False
        else:
            # Unreceived branches
            pending_branches = [key for key, neighbor in branches.items() if neighbor.get('rx', 0) == 0]
            self.log(f"WAITING FOR: {pending_branches}")


    def start_reply(self):
        '''The reply process'''

        if self.start_flag:
            # Check if edge node or reply timeout exceeded
            if self.is_edge_node() or (utime.time() - self.last_request_time > REPLY_TIMEOUT + 5):
                # Only add own data if not already added
                if self.node_id not in self.data_cache:
                    self.data_cache_update()  # Add own data to the data cache

                # Log the neighbor table entries where 'path' == self.node_id
                branches = {key: neighbor for key, neighbor in self.neighbor_table.items() if neighbor.get('path') == self.node_id}

                # Check if all branches have 'rx' == 1
                if all(neighbor.get('rx', 0) == 1 for neighbor in branches.values()):
                    if self.node_id != GATEWAY_MAC:
                        # utime.sleep(self.rand_delay())  # Wait for a random delay
                        self.send_dreply()  # Send the data packet to the next node
                    if self.node_id == GATEWAY_MAC:
                        self.log_data()  # Log the collected data
                        self.log(f"DATA:\n" + self.init_data_cache())
                        self.log(f"COMPLETE")
                        self.tx_counter += 1
                        self.start_seq = True
                        self.neighbor_table = {}  # Reset the Gateway neighbor table
                        self.data_cache = {}  # Reset the data cache
                        utime.sleep(INTERVAL)
                self.start_flag = False

    def calculate_overhead(self, sender_mac, data):
        '''
        Calculate the overhead based on battery level and RSSI.
        '''
        # Parse the battery and RSSI values as floats
        received_apparent_battery = float(data['b'])
        received_apparent_rssi = float(data['r'])

        # Apparent normalized battery level and RSSI
        # Normalize the battery voltage to a level between 0 and 1
        # Assuming battery voltage ranges from 3.0V (0%) to 4.2V (100%)
        battery_level = (self.read_battery_level() - 3.0) / (4.2 - 3.0)
        battery_level = min(max(battery_level, 0.0), 1.0)  # Clamp between 0 and 1
        self.apparent_battery = battery_level + received_apparent_battery

        # Normalize the RSSI to a level between 0 and 1
        # Assuming RSSI ranges from -124 (0%) to 0 (100%)
        rssi = (self.read_rssi(sender_mac) - (-124)) / (0 - (-124))
        self.apparent_rssi = rssi + received_apparent_rssi

        # Calculate and return the overhead
        overhead = round(WEIGHT * self.apparent_battery + (1 - WEIGHT) * -1 * self.apparent_rssi, 3)
        return overhead

    def is_new_transaction(self, data):
        '''If new transaction, reset the node.'''
        if data['tx'] != self.tx_counter:
            self.tx_counter = data['tx']
            self.neighbor_table = {}
            self.battery_level = 0.0
            self.rssi = 0.0
            self.apparent_battery = 0
            self.apparent_rssi = 0.0
            self.overhead = 0.0
            self.path = GATEWAY_MAC
            self.start_flag = False
            self.data_cache = {}

    def is_edge_node(self):
        '''
        Determine if the node is an edge node by checking the path of all neighbors.
        The node is an edge node if its ID is not stored as the path in any neighbor's entry.
        Returns True if the node is an edge node, False otherwise.
        '''
        for neighbor in self.neighbor_table.values():
            if neighbor.get('path') == self.node_id:
                return False
        return True

    def lowest_overhead_neighbor(self):
        '''
        Returns the neighbor with the lowest overhead.
        '''
        return min(self.neighbor_table, key=lambda n: self.neighbor_table[n]['overhead'])

    def update_neighbor_table(self, sender_mac, data):
        '''Update the neighbor table with sender info and add peer to ESP-NOW'''

        mac_bytes = bytes(int(b, 16) for b in sender_mac.split(':'))

        try:
            self.esp.add_peer(mac_bytes)  # Attempt to add the peer
            # self.log(f"Added peer {sender_mac}")
        except OSError as e:
            if e.args[0] == -12395:  # ESP_ERR_ESPNOW_EXIST
                pass  # Peer already exists, no action needed
            else:
                self.log(f"Error adding peer {sender_mac}: {e}")

        self.neighbor_table[sender_mac] = {
            'battery_level': float(data['b']),  # Store the battery level
            'rssi': float(data['r']),           # Store the RSSI
            'overhead': float(data['o']),       # Store the overhead
            'path': data['p'],                  # Store the path
            'rx': 0                             # Initialize packets received
        }

    def data_cache_update(self):
        '''
        Update the data cache with the collected data.
        '''

        # Get GPS data
        self.update_gps()

        # Read environmental data from BME280, if none set to 0
        try:
            temperature, pressure, humidity = self.read_bme280_data()
        except Exception:
            temperature, pressure, humidity = 0.0, 0.0, 0.0

        # Read actual battery level
        battery_level = self.read_battery_level()

        # Read RSSI value
        rssi = self.read_rssi(self.path)

        self.data_cache[str(self.node_id)] = {
            'c': self.now,
            'i': str(self.node_id),
            'P': [round(coord, 6) for coord in self.position],  # Limit precision
            't': round(temperature, 1),
            'h': int(round(humidity)),     # Integer percentage
            'p': int(round(pressure)),     # Integer hPa
            'b': round(battery_level, 1),  # Battery voltage with 1 decimal places
            'r': int(rssi),
            'H': self.hop_count
        }

    def update_gps(self):
        '''
        Fetch GPS data for a period of DELAY seconds, cache unique NMEA sentences, and update the position.
        '''
        gps_buffer = b''  # Buffer to collect raw data
        cached_sentences = {}  # Dictionary to store unique NMEA sentences

        start_time = utime.time()  # Record the start time
        while utime.time() - start_time < DELAY:
            # Check if there is data available from the GPS
            if self.gps_uart.any():
                data = self.gps_uart.read()
                if data:
                    gps_buffer += data  # Accumulate raw data

                    # Check if a full NMEA sentence is available (ends with newline)
                    while b'\n' in gps_buffer:
                        # Split the buffer at the newline to get a full sentence
                        nmea_sentence, gps_buffer = gps_buffer.split(b'\n', 1)

                        try:
                            # Decode the sentence and strip extra characters
                            nmea_sentence = nmea_sentence.decode('utf-8').strip()

                            # Only add valid NMEA sentences (those that start with '$')
                            if nmea_sentence.startswith('$'):
                                sentence_type = nmea_sentence.split(',')[0]  # Get the sentence type (e.g., $GNGGA)

                                # Overwrite previous sentence of the same type
                                cached_sentences[sentence_type] = nmea_sentence
                        except UnicodeError:
                            self.log("Failed to decode GPS sentence")

        # Process the cached NMEA sentences after DELAY seconds
        self.process_cached_nmea(list(cached_sentences.values()))

    def handle_pps_pulse(self, pin):
        '''
        Interrupt handler for PPS pulse.
        '''
        current_time = utime.ticks_us()
        if hasattr(self, 'last_pps_time'):
            # Ignore pulses that occur too close together (e.g., within 500 ms)
            if utime.ticks_diff(current_time, self.last_pps_time) < 500000:
                return
        self.last_pps_time = current_time

        # Synchronize system time if GPS time is available
        if hasattr(self, 'gps_time'):
            # Set the system time to the last known GPS time plus one second
            rtc = RTC()
            year, month, day, hours, minutes, seconds = self.gps_time
            seconds += 1  # PPS indicates the start of the next second
            if seconds >= 60:
                seconds = 0
                minutes += 1
            if minutes >= 60:
                minutes = 0
                hours += 1
            if hours >= 24:
                hours = 0
                day += 1  # Simplified; does not handle month/year rollover

            rtc.datetime((year, month, day, 0, hours, minutes, seconds, 0))
            # self.log(f"System time synchronized using PPS: {hours:02}:{minutes:02}:{seconds:02} on {day:02}/{month:02}/{year}")

    def process_cached_nmea(self, cached_sentences):
        '''
        Process cached NMEA sentences to extract location data dynamically and select the best fix.
        '''
        best_fix = None
        best_hdop = float('inf')  # Initialize with the worst HDOP

        for sentence in cached_sentences:
            if sentence.startswith('$GNGGA') or sentence.startswith('$GPGGA'):
                # Parse $GNGGA or $GPGGA sentence for fix quality, number of satellites, and HDOP
                try:
                    parts = sentence.split(',')
                    latitude = self.convert_to_degrees(parts[2], parts[3])
                    longitude = self.convert_to_degrees(parts[4], parts[5])
                    fix_quality = int(parts[6])  # Fix quality: 0 = Invalid, 1 = GPS fix, 2 = DGPS fix
                    num_satellites = int(parts[7])
                    hdop = float(parts[8]) if parts[8] else float('inf')  # Horizontal Dilution of Precision

                    # self.log(f"GGA - Lat: {latitude}, Lon: {longitude}, Fix: {fix_quality}, Satellites: {num_satellites}, HDOP: {hdop}")

                    # Only consider valid fixes
                    if fix_quality > 0 and hdop < best_hdop:
                        best_fix = {
                            'latitude': latitude,
                            'longitude': longitude,
                            'fix_quality': fix_quality,
                            'num_satellites': num_satellites,
                            'hdop': hdop
                        }
                        best_hdop = hdop

                except (ValueError, IndexError) as e:
                    #self.log(f"Error parsing GGA: {str(e)}")
                    pass

            elif sentence.startswith('$GNRMC') or sentence.startswith('$GPRMC'):
                # Parse $GNRMC or $GPRMC sentence for latitude, longitude, fix status, and time
                try:
                    parts = sentence.split(',')
                    fix_status = parts[2]  # A = Valid, V = Invalid
                    latitude = self.convert_to_degrees(parts[3], parts[4])
                    longitude = self.convert_to_degrees(parts[5], parts[6])

                    # self.log(f"RMC - Lat: {latitude}, Lon: {longitude}, Fix: {fix_status}")

                    # Extract GPS time (UTC) from RMC sentence
                    gps_time = parts[1]  # hhmmss.sss format
                    gps_date = parts[9]  # ddmmyy format

                    if gps_time and gps_date:
                        hours = int(gps_time[0:2])
                        minutes = int(gps_time[2:4])
                        seconds = int(gps_time[4:6])

                        day = int(gps_date[0:2])
                        month = int(gps_date[2:4])
                        year = 2000 + int(gps_date[4:6])  # Assuming 20xx

                        # Set the system time using the GPS time
                        rtc = RTC()
                        rtc.datetime((year, month, day, 0, hours, minutes, seconds, 0))  # Last 0 is for subseconds

                        # Store GPS time for PPS synchronization
                        self.gps_time = (year, month, day, hours, minutes, seconds)

                        # self.log(f"System time updated from GPS: {hours:02}:{minutes:02}:{seconds:02} on {day:02}/{month:02}/{year}")

                    # Consider RMC data if it's valid and there's no better GGA fix
                    if fix_status == 'A' and best_fix is None:
                        best_fix = {
                            'latitude': latitude,
                            'longitude': longitude,
                            'fix_quality': 1,  # Assume basic GPS fix
                            'num_satellites': 0,  # Not provided in RMC
                            'hdop': best_hdop  # No HDOP in RMC, so keep the current best
                        }

                except (ValueError, IndexError) as e:
                    #self.log(f"Error parsing RMC: {str(e)}")
                    pass

        # Use the best fix if found
        if best_fix:
            self.gps_flag = True
            self.position = [best_fix['latitude'], best_fix['longitude']]
            self.log(f"GPS fix acquired: Latitude = {best_fix['latitude']}, Longitude = {best_fix['longitude']}, Fix Quality = {best_fix['fix_quality']}, Satellites = {best_fix['num_satellites']}, HDOP = {best_fix['hdop']}")
        else:
            self.gps_flag = False
            self.position = [0.0, 0.0]
            self.log("No valid GPS fix found.")

    def convert_to_degrees(self, raw_value, direction):
        '''
        Convert NMEA format to degrees (for latitude/longitude).
        '''
        if not raw_value or not direction:
            raise ValueError("Invalid NMEA data")

        raw_value = float(raw_value)
        if direction in ['N', 'S']:  # Latitude
            degrees = int(raw_value / 100)
            minutes = raw_value - degrees * 100
        elif direction in ['E', 'W']:  # Longitude
            degrees = int(raw_value / 100)
            minutes = raw_value - degrees * 100
        else:
            raise ValueError(f"Invalid direction: {direction}")

        result = degrees + minutes / 60
        if direction in ['S', 'W']:
            result = -result
        return result

    def read_bme280_data(self):
        '''
        Read temperature, humidity, and pressure from the BME280 sensor.
        '''
        if self.bme:
            temperature = float(self.bme.temperature[:-1])  # Remove 'C' at the end
            humidity = float(self.bme.humidity[:-1])       # Remove '%' at the end
            pressure = float(self.bme.pressure[:-3])       # Remove 'hPa' at the end
            return temperature, pressure, humidity
        else:
            raise Exception("BME280 sensor not initialized")

    def read_battery_level(self):
        '''
        Read the battery level from an ADC pin.
        '''
        raw_value = 0
        for _ in range(10):
            raw_value += self.adc.read()
        raw_value /= 10

        # self.log(f"Raw ADC value: {raw_value}")

        v_ref = (3.3 / 4096.0) * ((33 + 82) / 82)

        # Calculate the actual battery voltage
        self.battery_level = raw_value * v_ref

        return self.battery_level

    def read_rssi(self, sender_mac):
        '''
        Read the RSSI value for a specific MAC address or the connected Wi-Fi network.
        '''
        mac_bytes = bytes(int(b, 16) for b in sender_mac.split(':'))

        if mac_bytes and mac_bytes in self.esp.peers_table:
            self.rssi = self.esp.peers_table[mac_bytes][0]  # RSSI value
        else:
            self.rssi = self.sta.status('rssi')  # RSSI of connected Wi-Fi network

        return self.rssi

    def log(self, msg):
        print(f"Node {str(self.node_id[12:]):6}[{self.now}] {msg}")

    def log_data(self):
        '''Log the collected data'''

        # Only add own data if not already added
        if self.node_id not in self.data_cache:
            self.data_cache_update()  # Add own data to the data cache

        # Calculate latency in seconds
        self.latency = utime.ticks_diff(utime.ticks_ms(), self.latency) / 1000

        # Add data to the log file
        latency_filename = 'latency.csv'
        try:
            if self.tx_counter == 0:
                # Overwrite the file if tx_counter is 0
                mode = 'w'
                data_string = "Tx,Latency\n"
                data_string += f"{self.tx_counter},{self.latency:.2f}\n"
            else:
                # Append to the file if tx_counter is not 0
                mode = 'a'
                data_string = f"{self.tx_counter},{self.latency:.2f}\n"

            with open(latency_filename, mode) as f:
                f.write(data_string)
            # self.log(f"Latency data written to {latency_filename}")
        except Exception as e:
            self.log(f"Error writing latency data to file: {str(e)}")

        # Write data cache to CSV file
        data_filename = 'data_log_SIM.csv'
        try:
            if self.tx_counter == 0:
                # Overwrite the file if tx_counter is 0
                mode = 'w'
                data_string = self.init_csv_data_cache()
            else:
                # Append to the file if tx_counter is not 0
                mode = 'a'
                data_string = self.format_csv_data_cache()

            with open(data_filename, mode) as f:
                f.write(data_string + '\n')
            # self.log(f"Data written to {data_filename}")
        except Exception as e:
            self.log(f"Error writing data to file: {str(e)}")


        

    def init_csv_data_cache(self):
        '''
        Returns a formatted string representation of the data cache in CSV format.
        '''
        formatted_cache = []
        header = (
            f"Tx,Time,MAC,PosX,PosY,Temp,Hum,Pres,Batt,RSSI,HopCount"
        )
        formatted_cache.append(header)

        for id, data in self.data_cache.items():
            pos_x, pos_y = data['P']  # Unpack position into x and y
            formatted_data = (
                f"{self.tx_counter}," +
                f"{data['c']}," +
                f"{id}," +
                f"{pos_x}," +
                f"{pos_y}," +
                f"{data['t']:.1f}," +
                f"{data['h']}," +
                f"{data['p']}," +
                f"{data['b']:.1f}," +
                f"{data['r']}," +
                f"{data['H']}"
            )
            formatted_cache.append(formatted_data)

        return "\n".join(formatted_cache)

    def format_csv_data_cache(self):
        '''
        Returns a formatted string representation of the data cache in CSV format.
        '''
        formatted_cache = []

        for id, data in self.data_cache.items():
            pos_x, pos_y = data['P']  # Unpack position into x and y
            formatted_data = (
                f"{self.tx_counter}," +
                f"{data['c']}," +
                f"{id}," +
                f"{pos_x}," +
                f"{pos_y}," +
                f"{data['t']:.1f}," +
                f"{data['h']}," +
                f"{data['p']}," +
                f"{data['b']:.1f}," +
                f"{data['r']}," +
                f"{data['H']}"
            )
            formatted_cache.append(formatted_data)

        return "\n".join(formatted_cache)

    def init_data_cache(self):
        '''
        Returns a formatted string representation of the data cache with aligned columns.
        '''
        formatted_cache = []
        header = (
            f"Data Log:\n" +
            f"{'Tx':<4}" +
            f"{'Time':<11}" +
            f"{'MAC':<15}" +
            f"{'Position':<22}" +
            f"{'Temp':<8}" +
            f"{'Hum':<8}" +
            f"{'Pressure':<12}" +
            f"{'Batt(V)':<8}" +
            f"{'RSSI':<6}" +
            f"{'HopCount':<6}"
        )
        formatted_cache.append(header)
        formatted_cache.append('-' * 100)  # Divider line

        for node_id, data in self.data_cache.items():
            formatted_data = (
                f"{self.tx_counter:<4}" +
                f"{data['c']:<11}" +
                f"{node_id[12:]:<15}" +
                f"{str(data['P']):<22}" +
                f"{data['t']:<8.1f}" +
                f"{data['h']:<8}" +
                f"{data['p']:<12}" +
                f"{data['b']:<8.1f}" +
                f"{data['r']:<6}" +
                f"{data['H']:<6}"
            )
            formatted_cache.append(formatted_data)

        return "\n".join(formatted_cache)

    def send_data_over_gsm(self, data):
        '''
        Send the collected data over the GSM module.
        '''
        if self.gsm_uart:
            try:
                # Example of sending data via GSM module
                # This is a placeholder for actual GSM communication code
                # For instance, sending an HTTP POST request via GSM module

                # Initialize GSM module commands here
                self.log("Initializing GSM module for data transmission...")

                # Send AT commands to establish connection and send data
                # This is a simplified example; actual implementation may vary
                self.gsm_uart.write('AT\r\n')
                utime.sleep(1)
                response = self.gsm_uart.read()
                self.log(f"GSM Response: {response}")

                # Additional GSM communication logic goes here
                # For example, sending data to a server using HTTP over GSM

                self.log("Data sent over GSM module.")
            except Exception as e:
                self.log(f"Error sending data over GSM: {str(e)}")
        else:
            self.log("GSM module not initialized or not present on this node.")

    @property
    def now(self):
        # Get the current local time in readable format (e.g., HH:MM:SS)
        current_time = utime.localtime()
        return "{:02}:{:02}:{:02}".format(current_time[3], current_time[4], current_time[5])

    def rand_delay(self):
        '''
        Returns a random delay between 0.2 and 0.8 seconds.
        '''
        return random.uniform(0.2, 0.8)

###########################################################
if __name__ == "__main__":
    run()
