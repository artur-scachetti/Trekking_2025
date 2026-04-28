#!/usr/bin/env python3

import rospy
import serial
import struct
import threading

from robot_control.msg import UARTData, VelocityData

SERIAL_PORT = "/dev/ttyUSB0"                    # OU /ttyACM0
BAUD_RATE = 115200

FRAME_SOF = 0xAA
FRAME_EOF = 0xBB

PAYLOAD_SIZE_RX = 16                            # 3 int + 1 uint32
FRAME_SIZE_RX = 1 + PAYLOAD_SIZE_RX + 1 + 1     # 19 bytes

PAYLOAD_SIZE_TX = 12                            # 3 int
FRAME_SIZE_TX = 1 + PAYLOAD_SIZE_TX + 1 + 1     # 15 bytes

SERVO_INITIAL_ANGLE = 90.0

class UARTDevice:
    def __init__(self):

        self.ackr_commands = [0, 0, SERVO_INITIAL_ANGLE]
        

        self.pub_encoder = rospy.Publisher('/uart_data', UARTData, queue_size=10)
        self.sub_velocity = rospy.Subscriber('/velocity_command', VelocityData, self.callback_vel)


        try:
            self.serial_port = serial.Serial(
                port=SERIAL_PORT,
                baudrate=BAUD_RATE,
                timeout=0.1 
            )
            rospy.loginfo(f"Conectado à porta serial: {SERIAL_PORT} a {BAUD_RATE} bps")
        except serial.SerialException as e:
            rospy.logerr(f"Falha ao abrir porta serial: {e}")
            exit(1)

        self.running = True
        self.read_thread = threading.Thread(target=self.update)
        self.read_thread.daemon = True
        self.read_thread.start()

    def calculate_checksum(self, data_bytes):
        """
        Realiza a soma simples dos bytes e retorna o byte menos significativo
        """
        return sum(data_bytes) & 0xFF

    def callback_vel(self, msg):
        
        self.ackr_commands[0] = msg.angular_speed_left
        self.ackr_commands[1] = msg.angular_speed_right
        self.ackr_commands[2] = msg.servo_angle

    def read_loop(self):
        """
        Loop contínuo para ler dados vindos do ESP32
        """
        expected_sof = struct.pack('B', FRAME_SOF)
        expected_eof = struct.pack('B', FRAME_EOF)

        while self.running and not rospy.is_shutdown():
            try:
                if self.serial_port.in_waiting > 0:
                    byte = self.serial_port.read(1)
                    
                    if byte == expected_sof:
                        rest_of_frame = self.serial_port.read(FRAME_SIZE_RX - 1)
                        
                        if len(rest_of_frame) == (FRAME_SIZE_RX - 1):
                            payload = rest_of_frame[0:16]
                            received_chk = rest_of_frame[16:17]
                            received_eof = rest_of_frame[17:18]
                            
                            if received_eof != expected_eof:
                                rospy.logwarn("Erro de EOF na serial")
                                continue

                            calculated_chk_int = self.calculate_checksum(payload)
                            calculated_chk_byte = struct.pack('B', calculated_chk_int)

                            if received_chk != calculated_chk_byte:
                                rospy.logwarn("Erro de Checksum na serial")
                                continue

                            x, y, z, timestamp = struct.unpack('<iiiI', payload)
                            self.publish_encoders(x, y, z, timestamp)
                            
            except Exception as e:
                rospy.logerr(f"Erro na leitura serial: {e}")
                rospy.sleep(1)

    def publish_encoders(self, x, y, z, timestamp):
        msg = UARTData()
        msg.x = x
        msg.y = y
        msg.z = z
        msg.timestamp = timestamp
        self.pub_encoder.publish(msg)

    def write_data(self):

        try:
            payload = struct.pack('<fff', self.ackr_commands[0], self.ackr_commands[1], self.ackr_commands[2])

            checksum = self.calculate_checksum(payload)

            frame = struct.pack('B', FRAME_SOF) + payload + struct.pack('B', checksum) + struct.pack('B', FRAME_EOF)

            self.serial_port.write(frame)

            rospy.loginfo(f'Valores enviados: {self.ackr_commands}')


        except Exception as e:
            rospy.logerr(f"Erro na escrita: {str(e)}")
            return None
        
    def shutdown(self):
        self.running = False
        if self.serial_port.is_open:
            self.serial_port.close()

    def update(self):
        
        rate = rospy.Rate(50)

        while not rospy.is_shutdown():

            self.read_loop()
            self.write_data()
            rate.sleep()

if __name__ == "__main__":
    
    try:
        rospy.init_node('uart_comm', anonymous=True)
        uart_communication = UARTDevice()
        rospy.spin()

    except rospy.ROSInterruptException:
        pass