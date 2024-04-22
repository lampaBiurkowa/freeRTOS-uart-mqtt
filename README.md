potentiometer connected to stm32

DMA transfers value from ADC to memory

its transmitted via UART to ESP32

who saves the data in queue and in separate tasks publishes to MQTT broker

app on PC (in C#) reads the data

also local app sends another values grouped by colors: yellow & green

ESP32 receives messages via MQTT and blinks correct diodes if values are high enough

also it transmits the data via UART to STM32

STM32 receiver tasks sends the received data into proper queues

dedicated tasks save the values from queues to appropiate EEPROM (separate for green & yellow) via I2C

in intervals it prints last persisted (in EEPROM) data for the colors (by doing a read via I2C)
