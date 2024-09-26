import network
import espnow  # type: ignore
import utime
import random
import json
from machine import SoftI2C, Pin, UART, RTC, ADC
import BME280
import utelnetserver

GATEWAY_MAC = 'e8:31:cd:70:f1:6c'  # Gateway MAC address
INTERVAL = 10  # Interval for sending DREQ messages
DELAY = 2  # Delay for reading GPS data
WEIGHT = 0.5  # Weight for calculating overhead (battery level and RSSI)

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

        ssid = "your_ssid"
        password = "your_password"

        # Initialize WLAN
        self.sta = network.WLAN(network.STA_IF)
        self.sta.active(True)
        self.sta.connect(ssid, password)

        while not self.sta.isconnected():
            utime.sleep(0.1)
        print('Connected, IP:', self.sta.ifconfig()[0])

        # Start the Telnet server for remote terminal access
        utelnetserver.start()

        # Initialize ESP-NOW
        self.esp = espnow.ESPNow()
        self.esp.active(True)

        # Set self.node_id to the MAC address in readable format
        self.node_id = ':'.join(f'{b:02x}' for b in self.sta.config('mac'))

        # Add the broadcast address to the peer list for discovering receivers
        self.broadcast_mac = b'\xff\xff\xff\xff\xff\xff'
        self.esp.add_peer(self.broadcast_mac)

        # Initialize I2C
        i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=100000)

        # Initialize UART for GPS
        self.gps_uart = UART(1, baudrate=9600, tx=17, rx=16)  # Use UART1 for GPS (TX=GPIO17, RX=GPIO16)

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
            self.adc = None

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

        self.start_dreq = True
        self.last_request_time = 0  # Store the time when the request was sent
        self.tx_counter = 0
        self.neighbor_table = {}  # Dictionary to store neighbors, their overheads, and paths
        self.battery_level = 0.0
        self.rssi = 0.0
        self.overhead = 0.0
        self.path = GATEWAY_MAC  # Path to the gateway
        self.clock_offset = 0  # Clock offset for synchronization
        self.start_flag = False
        self.data_cache = {}
        self.position = (0.0, 0.0)  # Initialize position

    def run(self):
        '''Main loop for node'''

        while True:
            if self.node_id == GATEWAY_MAC and self.start_dreq:
                self.log("STARTING...")
                utime.sleep(1)
                self.send_dreq()
                self.start_dreq = False
                self.last_request_time = utime.time()

            # Listen for messages
            try:
                sender, data = self.esp.recv(0)  # Non-blocking receive
                if sender and data:
                    sender_mac = ':'.join(f'{b:02x}' for b in sender)
                    data = json.loads(data.decode('utf-8'))  # Decode the message
                    self.log(f"RECEIVED: {data['msg']} from {sender_mac}")
                    self.on_receive(sender_mac, data['msg'], data)
            except OSError:
                pass  # No data received

            # Check if time since request exceeds INTERVAL - 5 seconds and no reply received
            if self.node_id == GATEWAY_MAC and not self.start_dreq:
                current_time = utime.time()
                if current_time - self.last_request_time > INTERVAL - 5:
                    self.log(f"No reply received after {INTERVAL - 5} seconds.")
                    self.start_dreq = True

            self.start_reply()

            utime.sleep(0.1)  # Small delay to prevent tight loop

    def send_dreq(self):
        '''
        Send a Data Request (DREQ) message to the broadcast address.
        '''

        data = {
            'msg': 'dreq',
            'tx_counter': self.tx_counter,
            'battery_level': self.battery_level,
            'rssi': self.rssi,
            'overhead': self.overhead,
            'path': self.path,
            'clock': self.now
        }
        json_data = json.dumps(data).encode('utf-8')  # Encode data to JSON

        utime.sleep(self.rand_delay())  # Random delay to simulate network latency

        try:
            self.esp.send(self.broadcast_mac, json_data)
            self.log(f"SENT: DREQ from {self.node_id}")
        except Exception as e:
            self.log(f"Error sending message: {str(e)}")

    def send_dreply(self):
        '''
        Send a Data Reply (DREP) message to the next node in the path.
        '''

        data = {'msg': 'dreply', 'data': self.data_cache}
        json_data = json.dumps(data).encode('utf-8')  # Encode data to JSON

        mac_bytes = bytes(int(b, 16) for b in self.path.split(':'))

        utime.sleep(self.rand_delay())  # Wait for a random delay

        try:
            self.esp.send(mac_bytes, json_data)
            self.log(f"SENT: DREP to {self.path}")
        except Exception as e:
            self.log(f"Error sending DREP: {str(e)}")

    def on_receive(self, sender_mac, msg, data):
        '''
        Handle received messages and act based on the message type.
        '''
        if msg == 'dreq':
            self.is_new_transaction(data)  # If new transaction, reset the node

            if not self.neighbor_table:  # First DREQ message received
                self.update_neighbor_table(sender_mac, data)  # Update the neighbor table
                if self.node_id != GATEWAY_MAC:
                    self.path = sender_mac  # Update self.path
                    self.overhead = self.calculate_overhead(sender_mac, data) + data['overhead']  # Calculate node overhead
                    utime.sleep(self.rand_delay())
                    self.send_dreq()  # Forward the DREQ message
                self.start_flag = True  # Set the start flag to True

            else:  # Node has received a DREQ message before
                if sender_mac not in self.neighbor_table or data['overhead'] < self.neighbor_table[sender_mac]['overhead']:
                    self.update_neighbor_table(sender_mac, data)  # Update the neighbor table

                    # Check if the new received overhead is less than the current lowest overhead neighbor
                    lowest_neighbor = self.lowest_overhead_neighbor()
                    if data['overhead'] < self.neighbor_table[lowest_neighbor]['overhead']:
                        if self.node_id != GATEWAY_MAC:
                            self.path = sender_mac  # Update self.path
                            self.overhead = self.calculate_overhead(sender_mac, data) + data['overhead']  # Update node overhead
                            utime.sleep(self.rand_delay())
                            self.send_dreq()  # Forward the DREQ message
                        self.start_flag = True

        elif msg == 'dreply':
            self.log(f"RECEIVED: DREP from {sender_mac}")

            self.data_cache_update()  # Add own data to the data cache

            self.data_cache.update(data['data'])  # Merge received data into the cache

            self.neighbor_table.setdefault(sender_mac, {})['rx'] = 1  # Update neighbor packets received

            # Log the neighbor table entries where 'path' == self.node_id
            branches = {key: neighbor for key, neighbor in self.neighbor_table.items() if neighbor.get('path') == self.node_id}

            # Check if all branches have 'rx' == 1
            if all(neighbor.get('rx', 0) == 1 for neighbor in branches.values()):
                if self.node_id != GATEWAY_MAC:  # If this node is not the gateway, forward the data packet
                    utime.sleep(self.rand_delay())  # Wait for a random delay
                    self.send_dreply()  # Send the data packet to the next node
                else:  # Data has reached the gateway
                    self.log("RECEIVED data from connected nodes")
                    self.log(self.format_data_cache())

                    # Send data over GSM module
                    self.send_data_over_gsm(self.format_data_cache())

                    utime.sleep(INTERVAL)  # Wait for INTERVAL seconds before sending the next DREQ message
                    self.log("COMPLETE")
                    self.start_dreq = True
                    self.tx_counter += 1
                    self.neighbor_table = {}  # Reset the gateway neighbor table
                    self.data_cache = {}  # Reset data cache
            else:
                # Unreceived branches
                pending_branches = [key for key, neighbor in branches.items() if neighbor.get('rx', 0) == 0]
                self.log(f"WAITING FOR: {pending_branches}")

    def start_reply(self):
        '''The reply process'''

        if self.is_edge_node() and self.start_flag and self.node_id != GATEWAY_MAC:
            self.data_cache_update()  # Add own data to the data cache
            self.send_dreply()  # Send the DREP message
            self.start_flag = False

    def lowest_overhead_neighbor(self):
        '''
        Returns the neighbor with the lowest overhead.
        '''
        return min(self.neighbor_table, key=lambda n: self.neighbor_table[n]['overhead'])

    def calculate_overhead(self, sender_mac, data):
        '''
        Calculate the overhead based on battery level and RSSI.
        '''
        mac_bytes = bytes(int(b, 16) for b in sender_mac.split(':'))

        # Parse the battery and RSSI values as floats
        received_battery = float(data['battery_level'])
        received_rssi = float(data['rssi'])

        # Read actual battery level and RSSI
        self.battery_level = self.read_battery_level() + received_battery
        self.rssi = self.read_rssi(mac_bytes) + received_rssi

        # Normalize the values
        normalized_battery = self.normalize_battery(self.battery_level)
        normalized_rssi = self.normalize_rssi(self.rssi)

        # Calculate and return the overhead
        overhead = WEIGHT * normalized_battery + (1 - WEIGHT) * normalized_rssi
        return overhead

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

    def is_new_transaction(self, data):
        '''If new transaction, reset the node.'''
        if data['tx_counter'] != self.tx_counter:
            self.tx_counter = data['tx_counter']
            self.neighbor_table = {}
            self.battery_level = 0.0
            self.rssi = 0.0
            self.overhead = 0.0
            self.path = GATEWAY_MAC
            self.start_flag = False
            self.data_cache = {}

    def update_neighbor_table(self, sender_mac, data):
        '''Update the neighbor table with sender info and add peer to ESP-NOW'''

        mac_bytes = bytes(int(b, 16) for b in sender_mac.split(':'))

        if mac_bytes not in self.esp.peers_table:
            try:
                self.esp.add_peer(mac_bytes)  # Add the peer
            except OSError as e:
                self.log(f"Error adding peer {sender_mac}: {e}")

        self.neighbor_table[sender_mac] = {
            'battery_level': float(data['battery_level']),  # Store the battery level
            'rssi': float(data['rssi']),                   # Store the RSSI
            'overhead': float(data['overhead']),           # Store the overhead
            'path': data['path'],                          # Store the path
            'rx': 0                                        # Store the number of packets received
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
        rssi = self.read_rssi()

        self.data_cache[str(self.node_id)] = {
            'time': self.now,
            'id': str(self.node_id),
            'position': self.position,
            'temperature': temperature,
            'humidity': humidity,
            'pressure': pressure,
            'rssi': rssi,
            'battery_level': battery_level
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

                    self.log(f"GGA - Lat: {latitude}, Lon: {longitude}, Fix: {fix_quality}, Satellites: {num_satellites}, HDOP: {hdop}")

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
                    self.log(f"Error parsing GGA: {str(e)}")

            elif sentence.startswith('$GNRMC') or sentence.startswith('$GPRMC'):
                # Parse $GNRMC or $GPRMC sentence for latitude, longitude, fix status, and time
                try:
                    parts = sentence.split(',')
                    fix_status = parts[2]  # A = Valid, V = Invalid
                    latitude = self.convert_to_degrees(parts[3], parts[4])
                    longitude = self.convert_to_degrees(parts[5], parts[6])

                    self.log(f"RMC - Lat: {latitude}, Lon: {longitude}, Fix: {fix_status}")

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

                        self.log(f"System time updated from GPS: {hours:02}:{minutes:02}:{seconds:02} on {day:02}/{month:02}/{year}")

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
                    self.log(f"Error parsing RMC: {str(e)}")

        # Use the best fix if found
        if best_fix:
            self.position = (best_fix['latitude'], best_fix['longitude'])
            self.log(f"Best position acquired: Latitude = {best_fix['latitude']}, Longitude = {best_fix['longitude']}, Fix Quality = {best_fix['fix_quality']}, Satellites = {best_fix['num_satellites']}, HDOP = {best_fix['hdop']}")
        else:
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
        if self.adc:
            raw_value = self.adc.read()
            voltage = (raw_value / 4095.0) * 3.3 * 2  # Multiply by 2 due to voltage divider
            battery_level = (voltage - 3.0) / (4.2 - 3.0)  # Normalize between 0 (3.0V) and 1 (4.2V)
            battery_level = min(max(battery_level, 0.0), 1.0)  # Clamp between 0 and 1
            return battery_level
        else:
            return 0.0

    def read_rssi(self, mac_bytes=None):
        '''
        Read the RSSI value for a specific MAC address or the connected Wi-Fi network.
        '''
        if mac_bytes and mac_bytes in self.esp.peers_table:
            rssi = self.esp.peers_table[mac_bytes][0]  # RSSI value
        else:
            rssi = self.sta.status('rssi')  # RSSI of connected Wi-Fi network
        return rssi

    def normalize_battery(self, battery_level):
        '''
        Normalize the battery level between 0 and 1.
        '''
        return 1.0 - battery_level  # Lower battery level means higher overhead

    def normalize_rssi(self, rssi):
        '''
        Normalize RSSI value between 0 and 1.
        '''
        normalized = (rssi - (-100)) / (-50 - (-100))  # Assuming RSSI range -100 dBm to -50 dBm
        normalized = min(max(normalized, 0.0), 1.0)  # Clamp between 0 and 1
        return 1.0 - normalized  # Lower RSSI (farther away) means higher overhead

    def log(self, msg):
        print(f"Node {str(self.node_id[12:]):6}[{self.now}] {msg}")

    def format_data_cache(self):
        '''
        Returns a formatted string representation of the data cache with aligned columns.
        '''
        formatted_cache = []
        header = (
            f"Tx:{self.tx_counter} DATA:\n" +
            f"{'Time':<9}" +
            f"{'MAC':<17}" +
            f"{'Position':<22}" +
            f"{'Temp':<8}" +
            f"{'Hum':<8}" +
            f"{'Pressure':<12}" +
            f"{'Batt':<6}" +
            f"{'RSSI':<8}"
        )
        formatted_cache.append(header)
        formatted_cache.append('-' * 100)  # Divider line

        for node_id, data in self.data_cache.items():
            formatted_data = (
                f"{data['time']:<9}" +
                f"{node_id:<17}" +
                f"{str(data['position']):<22}" +
                f"{data['temperature']:<8.2f}" +
                f"{data['humidity']:<8.2f}" +
                f"{data['pressure']:<12.2f}" +
                f"{data['battery_level']:<6.2f}" +
                f"{data['rssi']:<8}"
            )
            formatted_cache.append(formatted_data)

        formatted_cache.append('-' * 100)  # Divider line

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
        Returns a random delay between 0.2 and 0.6 seconds.
        '''
        return random.uniform(0.2, 0.6)

###########################################################
if __name__ == "__main__":
    run()
