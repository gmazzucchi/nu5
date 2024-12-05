#include "lcd_1602a/lcd_1602a.h"

#include "main.h"

#define N_DATA_LINES   (8)
#define CHARS_PER_LINE (16)
#define LINES          (2)

static GPIO_TypeDef *lcd1602a_ports[N_DATA_LINES] = {
    LCD1_D0_GPIO_Port,
    LCD1_D1_GPIO_Port,
    LCD1_D2_GPIO_Port,
    LCD1_D3_GPIO_Port,
    LCD1_D4_GPIO_Port,
    LCD1_D5_GPIO_Port,
    LCD1_D6_GPIO_Port,
    LCD1_D7_GPIO_Port,
};

const static uint16_t lcd1602a_pins[N_DATA_LINES] = {
    LCD1_D0_Pin,
    LCD1_D1_Pin,
    LCD1_D2_Pin,
    LCD1_D3_Pin,
    LCD1_D4_Pin,
    LCD1_D5_Pin,
    LCD1_D6_Pin,
    LCD1_D7_Pin,
};

const static uint8_t lcd_1602a_all_letters[26] = {
    0b10000010,  // A
    0b01000010,  // B
    0b11000010,  // C
    0b00100010,  // D
    0b10100010,  // E
    0b01100010,  // F
    0b11100010,  // G
    0b00010010,  // H
    0b10010010,  // I
    0b01010010,  // J
    0b11010010,  // K
    0b00110010,  // L
    0b10110010,  // M
    0b01110010,  // N
    0b11110010,  // O
    0b00001010,  // P
    0b10001010,  // Q
    0b01001010,  // R
    0b11001010,  // S
    0b00101010,  // T
    0b10101010,  // U
    0b01101010,  // V
    0b11101010,  // W
    0b00011010,  // X
    0b10011010,  // Y
    0b01011010,  // Z
};

const static uint8_t lcd_1602a_all_numbers[10] = {
    0b00001100,  // 0
    0b10001100,  // 1
    0b01001100,  // 2
    0b10001100,  // 3
    0b11001100,  // 4
    0b00101100,  // 5
    0b10101100,  // 6
    0b01101100,  // 7
    0b11101100,  // 8
    0b00011100,  // 9
};

static int display_pointer = 0;

void lcd_1602a_set_data_line_pins(int *states) {
    for (size_t i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(lcd1602a_ports[i], lcd1602a_pins[i], states[i]);
    }
}

void lcd_1602a_write_settings(int *states) {
    HAL_GPIO_WritePin(LCD1_RS_GPIO_Port, LCD1_RS_Pin, GPIO_PIN_RESET);        // instructions mode
    HAL_GPIO_WritePin(LCD1_ENABLE_GPIO_Port, LCD1_ENABLE_Pin, GPIO_PIN_SET);  // enable to talk with the LCD
    HAL_Delay(1);
    lcd_1602a_set_data_line_pins(states);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD1_ENABLE_GPIO_Port, LCD1_ENABLE_Pin, GPIO_PIN_RESET);
}

void lcd_1602a_clear_screen(void) {
    int lcd_1602a_clear_display[N_DATA_LINES] = {1, 0, 0, 0, 0, 0, 0, 0};
    lcd_1602a_write_settings(lcd_1602a_clear_display);
}

void lcd_1602a_init() {
    lcd_1602a_clear_screen();

    int lcd_1602a_return_home[N_DATA_LINES] = {0, 1, 0, 0, 0, 0, 0, 0};
    lcd_1602a_write_settings(lcd_1602a_return_home);

    int lcd_1602a_entry_mode[N_DATA_LINES] = {0, 1, 1, 0, 0, 0, 0, 0};
    lcd_1602a_write_settings(lcd_1602a_entry_mode);

    // int lcd_1602a_display_settings[N_DATA_LINES] = {1, 1, 1, 1, 0, 0, 0, 0};
    int lcd_1602a_display_settings[N_DATA_LINES] = {0, 0, 1, 1, 0, 0, 0, 0};
    lcd_1602a_write_settings(lcd_1602a_display_settings);

    int lcd_1602a_cursor_display_shift[N_DATA_LINES] = {0, 0, 1, 0, 1, 0, 0, 0};
    lcd_1602a_write_settings(lcd_1602a_cursor_display_shift);

    int lcd_1602a_functions_set[N_DATA_LINES] = {0, 0, 0, 1, 1, 1, 0, 0};
    lcd_1602a_write_settings(lcd_1602a_functions_set);

    int lcd_1602a_cgram[N_DATA_LINES] = {0, 0, 0, 0, 0, 0, 1, 0};
    lcd_1602a_write_settings(lcd_1602a_cgram);

    int lcd_1602a_ddram[N_DATA_LINES] = {0, 0, 0, 0, 0, 0, 0, 1};
    lcd_1602a_write_settings(lcd_1602a_ddram);
}

void lcd_1602a_select_line(int line) {
    if (line == 0) {
        int lcd_1602a_ddram[N_DATA_LINES] = {0, 0, 0, 0, 0, 0, 0, 1};
        lcd_1602a_write_settings(lcd_1602a_ddram);
    } else if (line == 1) {
        int lcd_1602a_ddram[N_DATA_LINES] = {0, 0, 0, 0, 0, 0, 1, 1};
        lcd_1602a_write_settings(lcd_1602a_ddram);
    }
    HAL_Delay(1);
}

void lcd_1602a_write_char(uint8_t c) {
    if (display_pointer == CHARS_PER_LINE) {
        lcd_1602a_select_line(1);
    }

    int states[N_DATA_LINES];
    for (size_t i = 0; i < N_DATA_LINES; i++) {
        states[i] = (c >> (N_DATA_LINES - i - 1)) & 1;
    }
    lcd_1602a_set_data_line_pins(states);
    HAL_GPIO_WritePin(LCD1_RS_GPIO_Port, LCD1_RS_Pin, GPIO_PIN_SET);  // data mode

    HAL_GPIO_WritePin(LCD1_ENABLE_GPIO_Port, LCD1_ENABLE_Pin, GPIO_PIN_SET);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD1_ENABLE_GPIO_Port, LCD1_ENABLE_Pin, GPIO_PIN_RESET);
    HAL_Delay(1);

    display_pointer++;
}

void lcd_1602a_write_text(const char *str) {
    lcd_1602a_clear_screen();
    while (*str) {
        if (*str == '#') { // sharp note
            lcd_1602a_write_char(0b11000100);
        } else if (*str == 'b') { // flat note
            lcd_1602a_write_char(0b01000110);
        } else if (*str >= 'A' && *str <= 'Z') {
            lcd_1602a_write_char(lcd_1602a_all_letters[*str - 'A']);
        } else if (*str >= '0' && *str <= '9') {
            lcd_1602a_write_char(lcd_1602a_all_numbers[*str - '0']);
        } else if (*str >= 'a' && *str <= 'z') {
            lcd_1602a_write_char(lcd_1602a_all_letters[*str - 'a']);
        } else {
            // TODO: add other chars, for now print a space
            lcd_1602a_write_char(0b00001000);
        }
        str++;  // += sizeof(char);
    }
    display_pointer = 0;
    lcd_1602a_select_line(0);
}

/* 
    void lcd_1602a_test() {
        int states[N_DATA_LINES] = {1, 0, 0, 0, 0, 0, 1, 0}; // letter A
        lcd_1602a_set_data_line_pins(states);
        HAL_GPIO_WritePin(LCD1_RS_GPIO_Port, LCD1_RS_Pin, GPIO_PIN_SET);        // data mode
    
        for (size_t iletter = 0; iletter < 16; iletter++) {
            HAL_GPIO_WritePin(LCD1_ENABLE_GPIO_Port, LCD1_ENABLE_Pin, GPIO_PIN_SET);  // enable to talk with the LCD
            HAL_Delay(1);
            HAL_GPIO_WritePin(LCD1_ENABLE_GPIO_Port, LCD1_ENABLE_Pin, GPIO_PIN_RESET);
            HAL_Delay(1);    
            HAL_Delay(200);
        }
    
        int lcd_1602a_ddram[N_DATA_LINES] = {0, 0, 0, 0, 0, 0, 1, 1};
        lcd_1602a_write_settings(lcd_1602a_ddram);
        HAL_Delay(1);
    
        lcd_1602a_set_data_line_pins(states);
        HAL_GPIO_WritePin(LCD1_RS_GPIO_Port, LCD1_RS_Pin, GPIO_PIN_SET);        // data mode
    
        for (size_t iletter = 0; iletter < 16; iletter++) {
            HAL_GPIO_WritePin(LCD1_ENABLE_GPIO_Port, LCD1_ENABLE_Pin, GPIO_PIN_SET);  // enable to talk with the LCD
            HAL_Delay(1);
            HAL_GPIO_WritePin(LCD1_ENABLE_GPIO_Port, LCD1_ENABLE_Pin, GPIO_PIN_RESET);
            HAL_Delay(1);    
            HAL_Delay(200);
        }
    } 
*/
