# Device Overview

This project is an electronic device based on the STM32 microcontroller, equipped with an Ethernet port for remote control and data communication. Designed for precise laser diode management, it offers a robust set of features for voltage measurement and control across two input and two output channels. This functionality enables it to monitor and adjust laser diode current and piezo voltage.

## Key Features

- **Voltage Measurement and Control**  
   - **Two Input Channels**: Measure input voltages for monitoring various parameters, such as optical cavity transmission or other analog signals relevant to laser operation.
   - **Two Output Channels**: Adjust output voltages to control the laser diode's current and piezo voltage, essential for stabilizing laser frequency and achieving desired experimental conditions.

- **Remote Operation via Ethernet**  
   The device can be controlled remotely over Ethernet, allowing users to access and manage laser parameters through a web interface. This setup facilitates seamless integration into laboratory environments, enabling convenient remote adjustments and monitoring.

- **Laser Relocking Algorithms**  
   Algorithms for laser relocking will be implemented directly within the STM32 microcontroller using C programming, allowing the device to function autonomously. In the event of laser instability, the device can automatically detect issues and initiate a relock process to maintain optimal performance.

- **Wavemeter Integration**  
   The Ethernet port will also facilitate communication with a wavemeter, enabling the device to maintain the laser at the desired experimental frequency. This feature is critical for applications requiring precise frequency stabilization and will allow the device to compensate for drift or fluctuations.

- **Autonomous Operation**  
   By implementing the control and relocking algorithms directly on the microcontroller, this device will operate autonomously, reducing the need for external computing resources. The integration with analog inputs for voltage monitoring further enhances its capacity to react to experimental changes and ensure stable laser operation without external intervention.

- **Sinara and ARTIQ Compatibility**  
   The device will be compatible with the Sinara system, and future integration with the ARTIQ control system is planned, making it highly suitable for advanced experimental setups and ensuring smooth operation within compatible environments.

## Applications and Use Cases

This device is intended for applications requiring remote control of laser parameters, particularly in environments where stability and autonomy are crucial. For example:

- **Optical Cavities**: During laser lock to an optical cavity, the device can monitor and maintain consistent transmission.
- **Frequency Stabilization**: Integration with a wavemeter allows for continuous frequency adjustment to meet experimental demands.
- **Autonomous Laser Maintenance**: The device can detect irregularities in laser performance and automatically initiate corrective actions, minimizing downtime and enhancing experimental reliability.

## Current Status

This project is under development, with the hardware currently in production. The documentation here provides an outline of the devices intended functionality, and further details will be added as development progresses.

## TCP232 Module Configuration

To start the board, you need to configure the TCP232 module as follows (for developers):

1. **Initial Connection**  
   When powered on for the first time, the TCP232 module operates at the default IP address `192.168.0.7` (subnet mask `/24`).  
   Set up your local network so you can communicate with this address.

2. **Accessing the Configuration Interface**  
   Connect to the module using a web browser by entering:  
   `http://192.168.0.7`  
   This will open the web interface, where you can change network settings, e.g., switch the module to DHCP.

3. **Serial Port Settings**  
   In the web interface, go to the **Serial Port** tab and set:
   - **Local Port Number**: `10`
   - **Work Mode**: `TCP Server`

After saving these settings, the module will be ready to work with the board in this project.

# License

[MIT](https://opensource.org/licenses/MIT)


# Acknowledgments
This work has received funding from the European Partnership on Metrology, co-financed by the European Union’s Horizon Europe Research and Innovation Programme and by the Participating States, under grant number 22IEM01 TOCK.

![badge](images/Acknowledgement%20badge.png)