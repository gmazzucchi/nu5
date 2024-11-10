#include "lcd_1602a/lcd_1602a.h"

#include "main.h"

#define N_DATA_LINES (8)

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

void lcd_1602a_set_data_line_pins(int *states) {
    for (size_t i = 0; i < 8; i++) {
        HAL_GPIO_WritePin(lcd1602a_ports[i], lcd1602a_pins[i], states[i]);
    }
}

void lcd_1602a_write_settings(int* states) {
    HAL_GPIO_WritePin(LCD1_RS_GPIO_Port, LCD1_RS_Pin, GPIO_PIN_RESET);        // instructions mode
    HAL_GPIO_WritePin(LCD1_ENABLE_GPIO_Port, LCD1_ENABLE_Pin, GPIO_PIN_SET);  // enable to talk with the LCD
    HAL_Delay(1);
    lcd_1602a_set_data_line_pins(states);
    HAL_Delay(1);
    HAL_GPIO_WritePin(LCD1_ENABLE_GPIO_Port, LCD1_ENABLE_Pin, GPIO_PIN_RESET);
}



void lcd_1602a_init() {
    int lcd_1602a_clear_display[N_DATA_LINES] = {1, 0, 0, 0, 0, 0, 0, 0};
    lcd_1602a_write_settings(lcd_1602a_clear_display);

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


