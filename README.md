# BlinkLED

This is a basic STM32CubeIDE project for the STM32F103C8T6 ("Blue Pill") microcontroller.  
It uses GPIO, UART, PWM, and a buzzer to demonstrate foundational embedded systems functionality.

## 🧠 Features

- **GPIO Input**: Reads user button (PC13)
- **GPIO Output**: Controls onboard LED (PA5)
- **PWM Output**: Drives piezo buzzer on PA6 (D12)
- **UART Communication**: Sends serial messages over USART2 (PA2/PA3)
- **Breadboard Circuitry**: Includes basic resistor/buzzer wiring
- **HAL Drivers**: All logic implemented using STM32 HAL in C
- **Debounce Logic**: Prevents button input noise with delay

## 🔌 Hardware Used

- STM32F103C8T6 "Blue Pill" board
- Onboard LED (PA5)
- User button (PC13)
- Piezo buzzer (12mm x 9mm, Same Sky)
- Breadboard + jumper wires
- 10kΩ resistor (used in buzzer circuit)
- UART-to-USB (via onboard ST-Link or external adapter)
- Serial terminal software (Putty)

## 🧪 How It Works

When the user button is pressed:
- The onboard LED toggles
- A buzzer beeps using PWM signal
- A "Button Pressed" message is sent over UART

## 🧭 Project Goals

This project is a stepping stone toward a larger "Smart Lock" embedded system project, which will include:
- Keypad input for PIN entry
- OLED screen for UI
- Logging access attempts
- Servo/motor control for physical locking
- Expansion to BLE, fingerprint, and RFID authentication

## 📚 Tools & Environment

- STM32CubeIDE (C / HAL)
- Git for version control
- Putty or any serial monitor for debugging

---

### ✅ Next Steps

- Add EEPROM or SD logging
- Add keypad input
- Integrate OLED screen for access feedback
- Begin modularizing code into reusable components
