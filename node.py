import network
import espnow  # type: ignore
import utime
import random
import json
from machine import SoftI2C, Pin, UART
import BME280

GATEWAY = 'e8:31:cd:70:f1:6c'  # Gateway MAC address
INTERVAL = 10  # Interval for sending DREQ messages
DELAY = 2 # Delay for reading GPS data
WEIGHT = 0.5  # Weight for calculating overhead (battery level and RSSI)

###########################################################
def run():
    '''
    Main function to run the Node.
    '''
    node = MyNode()
    node.init()
    node.run()  # Start the asynchronous event loop


###########################################################
class MyNode():

    ###################
    def init(self):
        '''
        Initialize the node.
        '''
        # Sender logic
        self.sta = network.WLAN(network.STA_IF)
        self.sta.active(True)
        self.sta.disconnect()

        # Initialize ESP-NOW
        self.esp = espnow.ESPNow()
        self.esp.active(True)

        # Set self.id to the MAC address in readable format
        self.id = ':'.join(f'{b:02x}' for b in self.sta.config('mac'))

        # Add the broadcast address to the peer list for discovering receivers
        self.broadcast_mac = b'\xff\xff\xff\xff\xff\xff'
        self.esp.add_peer(self.broadcast_mac)

        # Initialize I2C
        i2c = SoftI2C(scl=Pin(22), sda=Pin(21), freq=100000)

        # Initialize UART for GPS
        try:
            self.gps_uart = UART(1, baudrate=9600, tx=17, rx=16)  # Use UART1 for GPS (TX=GPIO17, RX=GPIO16)
            self.log("UART for GPS initialized successfully.")
        except OSError as e:
            self.log(f"OSError occurred during UART initialization: {str(e)}")
        except Exception as e:
            self.log(f"Unexpected error during UART initialization: {str(e)}")

        # Initialize BME280 sensor
        try:
            self.bme = BME280.BME280(i2c=i2c)  # Adjust address if necessary
            self.log("BME280 sensor initialized successfully.")
        except OSError as e:
            self.log(f"OSError occurred during BME280 sensor initialization: {str(e)}")
        except Exception as e:
            self.log(f"Unexpected error during BME280 sensor initialization: {str(e)}")

        self.pos = (0, 0)  # Initialize position to (0, 0)

        self.start_dreq = True
        self.txCounter = 0
        self.neighbor_table = {}  # Dictionary to store neighbors their overheads and path
        self.battery = 0
        self.rssi = 0
        self.overhead = 0
        self.path = GATEWAY  # Path to the gateway
        self.clk_offset = 0  # Clock offset for synchronization
        self.strt_flag = False
        self.dataCache = {}

    ###################
    def run(self):
        '''Main loop for Node'''

        while True:

            if self.id == GATEWAY and self.start_dreq:
                self.log(f"STARTING...")
                utime.sleep(1)
                self.send_dreq(self.txCounter, self.battery, self.rssi, self.overhead, self.path, self.now)
                self.start_dreq = False
            
            # Listen for tx
            try:
                sender, data = self.esp.recv()  # Listen for a message from any sender
                sender = ':'.join(f'{b:02x}' for b in sender)
            except Exception as e:
                self.log(f"Error receiving message: {str(e)}")

            if sender and data:
                data = json.loads(data.decode('utf-8'))  # Decode the message
                self.log(f"RECEIVED: {data['msg']} from {sender}")
                self.on_receive(sender, data['msg'], data)  # Handle the received message

            self.start_reply(sender)

    ###################
    def send_dreq(self, txCounter, battery, rssi, overhead, path, clk):
        '''
        Send a Data Request (DREQ) message to broadcast address
        '''
        
        data = {'msg': 'dreq', 'txCounter': txCounter, 'battery': battery, 'rssi': rssi, 'overhead': overhead, 'path': path, 'clk': clk}
        json_data = json.dumps(data).encode('utf-8')  # Encode data to JSON

        utime.sleep(randdelay())  # Random delay to simulate network latency

        
        try:
            self.esp.send(self.broadcast_mac, json_data)
            self.log(f"SENT: DREQ from {self.id}")
        except Exception as e:
            self.log(f"Error Sending message: {str(e)}")

    ###################
    def send_dreply(self, dataCache):
        '''
        Send a Data Reply (DREP) message to the lowest overhead node
        '''
        dataCache = {'msg': 'dreply', 'data': dataCache}
        json_data = json.dumps(dataCache).encode('utf-8')  # Encode data to JSON

        # Ensure self.path is a string
        if isinstance(self.path, int):
            self.path = str(self.path)
        mac_bytes = bytes(int(b, 16) for b in self.path.split(':'))

        # wait for a random delay then send the data packet
        utime.sleep(randdelay())
        self.esp.send(mac_bytes, json_data)
        self.log(f"SENT: DREP to {self.path}")


    ###################
    def on_receive(self, sender, msg, data):
        '''
        Handle received tx and act based on the message type
        '''
        if msg == 'dreq':
            self.isNewTx(data)  # If new transaction, reset the node

            if not self.neighbor_table:  # First DREQ message received
                self.updateNeighborTable(sender, data)  # Update the neighbor table                
                if self.id != GATEWAY:
                    self.path = sender  # Update self.path
                    self.overhead = self.calculate_overhead(sender, data) + data['overhead']  # Calculate node overhead
                    utime.sleep(randdelay())
                    self.send_dreq(self.txCounter, self.battery, self.rssi, self.overhead, self.path, self.now)  # Forward the DREQ message
                self.strt_flag = True  # Set the start flag to True

            if self.neighbor_table:  # Node has received a DREQ message before
                if sender not in self.neighbor_table or data['overhead'] < self.neighbor_table[sender]['overhead']:
                    self.updateNeighborTable(sender, data)  # Update the neighbor table

                    # Check if the new received overhead is less than the current lowest overhead neighbor
                    if data['overhead'] < self.neighbor_table[self.lowestoverheadNeighbor()]['overhead']:                        
                        if self.id != GATEWAY:
                            self.path = sender  # Update self.path
                            self.overhead = self.calculate_overhead(sender, data) + data['overhead']  # Update Node overhead
                            utime.sleep(randdelay())
                            self.send_dreq(self.txCounter, self.battery, self.rssi, self.overhead, self.path, self.now)  # Forward the DREQ message
                        self.strt_flag = True

        elif msg == 'dreply':
            self.log(f"RECEIVED: DREP from {sender}")

            self.dataCacheUpdate()  # Add own data to the dataCache

            self.dataCache.update(data['data'])  # Add the sent data to the dataCache by merging dictionaries

            self.neighbor_table.setdefault(sender, {})['rx'] = 1  # Update neighbor packets received

            # Log the neighbor table entries where 'path' == self.id
            branches = {key: neighbor for key, neighbor in self.neighbor_table.items() if neighbor.get('path') == self.id}

            # Check if all branches have 'rx' == 1
            if all(neighbor.get('rx', 0) == 1 for neighbor in branches.values()):
                if self.id != GATEWAY:  # If this node is not the gateway, forward the data packet
                    utime.sleep(randdelay())  # Wait for a random delay
                    self.send_dreply(self.dataCache)  # Send the data packet to the next node

                else:  # Data has reached the gateway
                    self.log(f"RECEIVED data from connected nodes")
                    self.log(self.format_data_cache())

                    utime.sleep(INTERVAL)  # Wait for INTERVAL seconds before sending the next DREQ message
                    self.log(f"COMPLETE")
                    self.start_dreq = True
                    self.txCounter += 1
                    self.neighbor_table = {} # Reset the Gateway neighbor table

            else:
                # Unreceived branches
                pendingBranches = [key for key, neighbor in branches.items() if neighbor.get('rx', 0) == 0]
                self.log(f"WAITING FOR: {pendingBranches}")

    ###################
    def start_reply(self, sender):
        '''The reply process'''

        if self.isEdgeNode() and self.strt_flag and self.id != GATEWAY:
            self.dataCacheUpdate()  # Add own data to the dataCache
            self.send_dreply(self.dataCache)  # Send the DREP message
            self.strt_flag = False


    ###################
    def lowestoverheadNeighbor(self):
        '''
        Returns the neighbor with the lowest overhead
        '''
        return min(self.neighbor_table, key=lambda n: self.neighbor_table[n]['overhead'])

    ###################
    def calculate_overhead(self, sender, data):
        '''
        Assign random values to battery level, RSSI, and distance to calculate the overhead.
        '''
        # Ensure sender is a string
        if isinstance(sender, int):
            sender = str(sender)
        mac_bytes = bytes(int(b, 16) for b in sender.split(':'))

        self.battery = (1-(random.randint(0, 100)/100))  + int(data['battery']) # battery level + accumulated battery level
        self.rssi = round((1-(self.esp.peers_table[mac_bytes][0]-(-127))/(0-(-127))) + int(data['rssi']),4)  # Normalized rssi + accumulated normalized rssi

        return round(WEIGHT*self.battery + (1-WEIGHT)*self.rssi)  # Calculate and return the overhead

    ###################
    def isEdgeNode(self):
        '''
        Determine if the node is an edge node by checking the path of all neighbors.
        The node is an edge node if its ID is not stored as the path in any neighbor's entry.
        Returns True if the node is an edge node, False otherwise.
        '''
        for neighbor in self.neighbor_table.values():
            if 'path' in neighbor and neighbor['path'] == self.id:
                return False
        return True

    ###################
    def isNewTx(self, data):
        '''If new transmission reset the node.'''
        if data['txCounter'] != self.txCounter:
            self.txCounter = data['txCounter']
            self.neighbor_table = {}
            self.battery = 0
            self.rssi = 0
            self.overhead = 1
            self.path = GATEWAY
            self.strt_flag = False
            self.dataCache = {}

    ###################
    def updateNeighborTable(self, sender, data):
        '''Update the neighbor tables with sender info and add peer to espnow'''

        # Ensure sender is a string
        if isinstance(sender, int):
            sender = str(sender)
        mac_bytes = bytes(int(b, 16) for b in sender.split(':'))

        try:
            self.esp.del_peer(mac_bytes)  # Delete the peer if it exists
        except OSError:
            pass  # Ignore if peer doesn't exist
        try:
            self.esp.add_peer(mac_bytes)  # Add the peer
        except OSError as e:
            print(f"Error adding peer {mac_bytes}: {e}")

        self.neighbor_table[sender] = {             # Update the neighbor table
            'battery': data['battery'],                 # Store the position
            'rssi': self.esp.peers_table[mac_bytes][0], # Store the RSSI
            'overhead': data['overhead'],               # Store the overhead
            'path': data['path'],                       # Store the path
            'rx': 0                                     # Store the number of packets received
        }


    ###################
    def dataCacheUpdate(self):
        '''
        Update the dataCache with the collected data.
        '''

        # Get GPS data
        self.update_gps()

        # Read environmental data from BME280, if none set to 0
        try:
            temperature, pressure, humidity = self.read_bme280_data()
        except Exception:
            temperature, pressure, humidity = 0, 0, 0
        
        self.dataCache[str(self.id)] = {
            'time': self.now,
            'id': str(self.id),
            'pos': str(self.pos),
            'temp': temperature,
            'hum': humidity,
            'pressure': pressure,
            'rssi': str(self.rssi),
            'battery': str(self.battery)
        }

    ###################
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
                    if b'\n' in gps_buffer:
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
                        except Exception:
                            self.log(f"Failed to decode GPS sentence")

        # Log the unique GPS sentences
        #self.log(f"GPS data: \n{list(cached_sentences.values())}")

        # Process the cached NMEA sentences after DELAY seconds
        self.process_cached_nmea(list(cached_sentences.values()))


    ###################
    def process_cached_nmea(self, cached_sentences):
        '''
        Process cached NMEA sentences to extract location data dynamically.
        '''
        latitude = None
        longitude = None
        fix_status = None
        altitude = None

        for sentence in cached_sentences:
            if sentence.startswith('$GNGGA'):
                # Parse $GNGGA sentence for latitude, longitude, and altitude
                try:
                    parts = sentence.split(',')
                    latitude = self.convert_to_degrees(parts[2], parts[3])
                    longitude = self.convert_to_degrees(parts[4], parts[5])
                    altitude = parts[9]
                    fix_status = parts[6]  # 0 = Invalid, 1 = GPS Fix, 2 = DGPS Fix
                    self.log(f"GNGGA - Lat: {latitude}, Lon: {longitude}, Altitude: {altitude}, Fix: {fix_status}")
                except Exception as e:
                    self.log(f"Error parsing GNGGA: {str(e)}")

            # elif sentence.startswith('$GNRMC'):
            #     # Parse $GNRMC sentence for latitude and longitude
            #     try:
            #         parts = sentence.split(',')
            #         latitude = self.convert_to_degrees(parts[3], parts[4])
            #         longitude = self.convert_to_degrees(parts[5], parts[6])
            #         fix_status = parts[2]  # A = Valid, V = Invalid
            #         self.log(f"GNRMC - Lat: {latitude}, Lon: {longitude}, Fix: {fix_status}")
            #     except Exception as e:
            #         self.log(f"Error parsing GNRMC: {str(e)}")

            # elif sentence.startswith('$GNGLL'):
            #     # Parse $GNGLL sentence for latitude and longitude
            #     try:
            #         parts = sentence.split(',')
            #         latitude = self.convert_to_degrees(parts[1], parts[2])
            #         longitude = self.convert_to_degrees(parts[3], parts[4])
            #         fix_status = parts[6]  # A = Valid, V = Invalid
            #         self.log(f"GNGLL - Lat: {latitude}, Lon: {longitude}, Fix: {fix_status}")
            #     except Exception as e:
            #         self.log(f"Error parsing GNGLL: {str(e)}")

        # Use the data if a valid fix is available
        if fix_status in ['1', 'A']:  # Check if the fix is valid
            self.pos = (latitude, longitude)
            self.log(f"Valid position acquired: Latitude = {latitude}, Longitude = {longitude}, Altitude = {altitude}")
        else:
            self.log("No valid GPS fix available.")


    ###################
    def convert_to_degrees(self, value, direction):
        '''
        Convert NMEA format to degrees (for latitude/longitude)
        '''
        degrees = float(value[:2])
        minutes = float(value[2:]) / 60
        result = degrees + minutes
        if direction in ['S', 'W']:
            result = -result
        return result

    ###################
    def read_bme280_data(self):
        '''
        Read temperature, humidity, and pressure from the BME280 sensor
        '''
        temperature = self.bme.temperature
        humidity = self.bme.humidity
        pressure = self.bme.pressure
        return temperature, pressure, humidity

    ############################
    def log(self, msg):
        print(f"Node {str(self.id):4}[{self.now:10}] {msg}")

    ############################
    def format_data_cache(self):
        '''
        Returns a formatted string representation of the data cache with aligned columns.
        '''
        formatted_cache = []
        header = (
            f"Tx:{self.txCounter} DATA:\n"+
            f"{'Time':<8}"+
            f"{'MAC':<6}"+
            f"{'Pos':<22}"+
            f"{'Temp':<8}"+
            f"{'Hum':<8}"+
            f"{'Pressure':<12}"+
            f"{'Battery':<8}"+
            f"{'RSSI':<8}"
        )
        formatted_cache.append(header)
        formatted_cache.append('-' * 80)  # Divider line

        for node_id, data in self.dataCache.items():
            formatted_data = (
                f"{data['time']:<8}"+
                f"{node_id[12:]:<6}"+
                f"{str(data['pos']):<22}"+
                f"{data['temp']:<8}"+
                f"{data['hum']:<8}"+
                f"{data['pressure']:<12}"+
                f"{data['battery']:<8}"+
                f"{data['rssi']:<8}"
            )
            formatted_cache.append(formatted_data)

        formatted_cache.append('-' * 80)  # Divider line

        return "\n".join(formatted_cache)

    ############################
    @property
    def now(self):
        return utime.ticks_ms()

###########################################################
def randdelay():
    '''
    Returns a random delay between 0.2 and 0.6 seconds.
    '''
    return random.uniform(0.2, 0.6)
