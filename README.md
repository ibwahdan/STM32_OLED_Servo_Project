# STM32 Secure Access System

This STM32F4 project demonstrates how to interface an SH1107 OLED display and a servo motor using button input on an STM32 Nucleo board. When a button is pressed, messages are displayed on the OLED, and the servo rotates accordingly. The project showcases basic use of:

I2C communication (OLED)

PWM output (servo and buzzer)

UART communication (debug messages to PC)

GPIO input handling (button)

This setup is a useful foundation for integrating visual feedback and actuator control into embedded systems.

---

## 🔧 Features

- ✅ **Button-Activated System** — Push-button press detected on PC13.
- 📟 **OLED Feedback** — Visual status updates like “LOCKED” or “UNLOCKED” via I2C OLED display (SH1107).
- 🔊 **Audible Alerts** — Buzzer provides feedback using PWM via TIM3 on PA6.
- 🔐 **Servo-Controlled Lock** — Servo motor rotates based on authentication status using TIM1 on PA9.
- 📤 **UART Logging** — Events like "Button Pressed" and "Unlocked Device" logged to PC via USART2.

---

## 🧰 Hardware Used

| Component            | Description                              |
|---------------------|------------------------------------------|
| STM32 Nucleo Board  | F401RE                                    |
| Push Button         | Onboard (PC13)                            |
| OLED Display        | 1.3" SH1107-based I2C (128x128)           |
| Servo Motor         | TowerPro SG90 (or similar 5V servo)       |
| Buzzer              | Standard active buzzer via PWM            |
| Jumper Wires        | Female-female and male-male as needed     |

---

## 🖧 Pin Configuration

| Peripheral     | STM32 Pin | Notes                  |
|----------------|-----------|------------------------|
| Button         | PC13      | Pull-up by default     |
| LED Indicator  | PA5       | Blinks on button press |
| OLED SCL       | PB8       | I2C1 Clock             |
| OLED SDA       | PB9       | I2C1 Data              |
| Buzzer (PWM)   | PA6       | TIM3_CH1               |
| Servo Motor    | PA9       | TIM1_CH2               |
| UART TX        | PA2       | USART2                 |
| UART RX        | PA3       | USART2                 |

---

## 🚦 Functional Flow

1. **User presses the button**
   - LED toggles
   - UART logs `"Button Pressed"`
   - OLED updates to `"LOCKED!"`
   - Buzzer sounds briefly
   - Servo moves to locked position

2. **After 5 presses**
   - OLED shows unlock animation
   - UART logs `"Unlocked Device"`
   - Buzzer beeps 5 times with servo movement
   - Servo rotates to center (unlocked) position

3. **After 10 presses**
   - Counter resets to 0

---

## 🧠 Custom Functions

| Function Name            | Purpose                                               |
|--------------------------|-------------------------------------------------------|
| `handle_button_press()`  | Toggles LED + debounces input                         |
| `send_uart_message()`    | Sends messages via USART2                             |
| `buzzer_on()` / `off()`  | Controls PWM duty cycle for buzzer                   |
| `servo_set_angle()`      | Converts angle to PWM pulse for servo control         |
| `SH1107_Init()`          | Initializes OLED with SH1107 command set              |
| `SH1107_Write_Data()`    | Sends a pixel byte to OLED                            |
| `SH1107_Write_Command()` | Sends command to OLED controller                      |
| `SH1107_Write_Char()`    | Draws a 5x8 character using custom bitmap font        |
| `SH1107_Write_String()`  | Displays a string by looping `Write_Char`             |
| `SH1107_Set_Cursor()`    | Positions OLED write location                         |
| `SH1107_Clear()`         | Clears entire screen                                  |
| `SH1107_Unlock_Animation()` | Flashes or scrolls OLED during unlock              |

---

## 📸 Screenshots / Media

| OLED Output          | Servo Position |
|----------------------|----------------|
| `LOCKED!`            | 180°           |
| `UNLOCKED!` + anim   | 90°            |

---

## 🚀 Future Features (Planned)

- 🔢 Keypad-based passcode system (awaiting hardware)
- 🧬 Fingerprint authentication using UART fingerprint module
- 🔐 Multi-level access modes (admin vs guest)
- 🔁 EEPROM to store state or passcode between power cycles

---

## 🤝 Acknowledgments

Built using STM32CubeIDE with STM32 HAL drivers.  
Assisted and debugged with the help of OpenAI's GPT-4.

---

